// =============================================================================
//  es_splitter.cpp —— ES 切帧实现
// =============================================================================
//  核心：扫 AnnexB 起始码（00 00 01 / 00 00 00 01），按 NAL 类型判断帧边界。
//
//  ★ 为什么不能"每个 VCL NAL 一帧"
//  一帧视频可以由**多个 slice** 组成（编码器做码率控制时切）。按 VCL 切
//  会切多：75 帧的流切成 147 帧，下游 DTS 重复。
//
//  正确的判据：
//    H.264：slice header 的第一个字段 first_mb_in_slice（ue(v)）== 0
//    H.265：slice segment header 的第一个 bit first_slice_segment_in_pic_flag == 1
// =============================================================================
#include <mpegmech/es_splitter.hpp>

#include <algorithm>
#include <cstring>
#include <vector>
#include <memory>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <utility>

namespace mpegmech {

namespace {

// =============================================================================
//  起始码
// =============================================================================

/// 起始码长度（3 或 4），不是起始码返回 0。
std::size_t startcode_len(const std::uint8_t* p, std::size_t avail) noexcept
{
    if (avail >= 4 && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) {
        return 4;
    }
    if (avail >= 3 && p[0] == 0 && p[1] == 0 && p[2] == 1) {
        return 3;
    }
    return 0;
}

// =============================================================================
//  位读取（只为解 first_mb_in_slice 的 ue(v)）
// =============================================================================

class BitReader {
public:
    BitReader(const std::uint8_t* data, std::size_t len) noexcept
        : m_data(data), m_len(len) {}

    /// 读 n 个 bit（n <= 32）。失败返回 false。
    bool read(std::size_t n, std::uint32_t& out) noexcept
    {
        out = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (m_bit >= m_len * 8) {
                return false;
            }
            const std::uint8_t byte = m_data[m_bit >> 3];
            out = (out << 1) | ((byte >> (7 - (m_bit & 7))) & 1u);
            ++m_bit;
        }
        return true;
    }

    /// 读一个 ue(v)（Exp-Golomb 无符号）。
    bool read_ue(std::uint32_t& out) noexcept
    {
        // 数前导 0
        std::size_t zeros = 0;
        while (true) {
            std::uint32_t b = 0;
            if (!read(1, b)) {
                return false;
            }
            if (b) {
                break;
            }
            if (++zeros > 31) {
                return false;   // 畸形
            }
        }
        if (zeros == 0) {
            out = 0;
            return true;
        }
        std::uint32_t rest = 0;
        if (!read(zeros, rest)) {
            return false;
        }
        out = ((1u << zeros) - 1) + rest;
        return true;
    }

private:
    const std::uint8_t* m_data;
    std::size_t         m_len;
    std::size_t         m_bit = 0;
};

// =============================================================================
//  NAL 分类
// =============================================================================

enum class NalKind {
    aud,        ///< 访问单元分隔符
    vcl_first,  ///< VCL 且是这一帧的第一个 slice
    vcl_more,   ///< VCL 但不是第一个 slice（同帧后续片）
    nonvcl,     ///< SPS/PPS/SEI 等
};

NalKind classify(std::uint8_t hdr, Codec codec, const std::uint8_t* payload,
                 std::size_t avail) noexcept
{
    if (codec == Codec::h265) {
        const std::uint8_t type = (hdr >> 1) & 0x3F;
        if (type == 35) {                 // AUD_NUT
            return NalKind::aud;
        }
        // VCL：0..31
        if (type <= 31) {
            // first_slice_segment_in_pic_flag 是 slice segment header 的第 1 个 bit
            if (avail == 0) {
                return NalKind::vcl_first;   // 没数据，保守当第一片
            }
            return (payload[0] & 0x80) ? NalKind::vcl_first : NalKind::vcl_more;
        }
        return NalKind::nonvcl;
    }

    // H.264
    const std::uint8_t type = hdr & 0x1F;
    if (type == 9) {                      // AUD
        return NalKind::aud;
    }
    if (type >= 1 && type <= 5) {         // VCL
        if (avail == 0) {
            return NalKind::vcl_first;
        }
        // first_mb_in_slice 是 slice header 的第一个字段（ue(v)）
        BitReader br(payload, avail);
        std::uint32_t first_mb = 0;
        if (!br.read_ue(first_mb)) {
            return NalKind::vcl_first;    // 解析失败，保守
        }
        return (first_mb == 0) ? NalKind::vcl_first : NalKind::vcl_more;
    }
    return NalKind::nonvcl;
}

} // namespace

// =============================================================================
//  Impl
// =============================================================================

struct EsSplitter::Impl {
public:
    explicit Impl(const Config& c) : cfg(c) {}

    Config cfg;
    Stats  stats;

    /// 未处理的输入。
    std::vector<std::uint8_t> in;

    /// 当前帧在 `in` 里的起点（还没到"下一帧起始码"）。
    std::size_t frame_start = 0;

    /// 扫描位置（已经判断过的起始码之后）。
    std::size_t scan_pos = 0;

    /// 是否已经丢弃过开头残片。
    bool dropped_leading = false;

    /// 当前帧里有没有 VCL（用于判断 SPS/PPS 该归谁）。
    bool frame_has_vcl = false;

    /// 丢掉 `in` 里 `frame_start` 之前的数据（那些已经输出过）。
    void compact()
    {
        if (frame_start == 0) {
            return;
        }
        if (frame_start >= in.size()) {
            in.clear();
            frame_start = 0;
            scan_pos    = 0;
            return;
        }
        in.erase(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(frame_start));
        scan_pos -= frame_start;
        frame_start = 0;
    }

    void emit(std::size_t end, const FrameCallback& on_frame)
    {
        if (end <= frame_start) {
            return;
        }
        const std::size_t len = end - frame_start;
        if (len > cfg.max_frame_bytes) {
            ++stats.oversize_drops;
            return;
        }
        ++stats.frames_out;
        // frame 是 in 的一个窗口，回调期间有效（in 不会被改动）
        on_frame(span<const std::uint8_t>(in.data() + frame_start, len));
    }

    std::error_code feed(span<const std::uint8_t> data, const FrameCallback& on_frame)
    {
        if (!data.empty()) {
            in.insert(in.end(), data.begin(), data.end());
            stats.bytes_in += data.size();
        }

        // 防御：一直喂不到起始码
        if (in.size() - frame_start > cfg.max_buffered_bytes) {
            ++stats.overflow_drops;
            const std::size_t keep = cfg.max_buffered_bytes / 2;
            const std::size_t drop = in.size() - keep;
            stats.no_startcode += drop;
            in.erase(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(drop));
            frame_start = 0;
            scan_pos    = 0;
            frame_has_vcl = false;
            return make_error_code(MpegErrc::buffer_overflow);
        }

        std::size_t i = scan_pos;
        while (true) {
            const std::size_t avail = in.size() - i;
            if (avail < 3) {
                break;                      // 可能半个起始码
            }
            const std::size_t sc = startcode_len(in.data() + i, avail);
            if (sc == 0) {
                ++i;
                continue;
            }

            const std::size_t nal_at = i + sc;
            if (nal_at >= in.size()) {
                break;                      // 起始码之后还没数据
            }

            const NalKind kind = classify(in[nal_at], cfg.codec, in.data() + nal_at + 1,
                                          in.size() - nal_at - 1);

            if (!dropped_leading) {
                // ★ 非开头输入：第一个起始码**不一定是帧开头** —— 也可能是
                //   某个 IDR 的第二片（vcl_more）。那种情况下从这里开始切，
                //   会多切出一个"残片帧"（实测：74 个 AUD 的流切出 75 帧）。
                //
                //   所以只接受能确定是帧开头的 NAL：
                //     AUD / vcl_first（slice 第一片）/ nonvcl（SPS/PPS 后面跟 IDR）
                //   vcl_more 继续往前扫。
                //   注意 **不包括 nonvcl（SPS/PPS）**：它可能属于"已经开始
                //   的那一帧"（有些流会在 slice 之后补发参数集），从它开始
                //   切同样会多出一个残片帧。
                const bool valid_start = (kind == NalKind::aud) ||
                                         (kind == NalKind::vcl_first);
                if (!valid_start) {
                    ++i;
                    continue;
                }
                if (i > 0 && cfg.drop_leading_partial) {
                    stats.dropped_leading_bytes += i;
                }
                dropped_leading = true;
                frame_start = i;
            } else if (frame_has_vcl) {
                // ★ 关键判据：只有**当前帧已经出现过 VCL**时，新来的
                //   AUD / vcl_first / SPS-PPS 才意味着"上一帧结束"。
                //
                //   不这么判会怎样：一帧是 [AUD][SPS][PPS][IDR slice]，
                //   如果在"还没有 VCL"时遇到 vcl_first 就切，[AUD] 会被
                //   单独切成一帧 —— 75 帧的流切成 150 帧。
                //
                //   vcl_more（同一帧的后续 slice）永远不切。
                const bool boundary = (kind == NalKind::aud) ||
                                      (kind == NalKind::vcl_first) ||
                                      (kind == NalKind::nonvcl);
                if (boundary && i > frame_start) {
                    emit(i, on_frame);
                    frame_start = i;
                    frame_has_vcl = false;
                }
            }

            if (kind == NalKind::vcl_first || kind == NalKind::vcl_more) {
                frame_has_vcl = true;
            }

            i = nal_at;
            scan_pos = i;
        }

        compact();
        return {};
    }

    std::error_code flush(const FrameCallback& on_frame)
    {
        emit(in.size(), on_frame);
        in.clear();
        frame_start = 0;
        scan_pos    = 0;
        frame_has_vcl = false;
        return {};
    }

    void reset() noexcept
    {
        in.clear();
        frame_start = 0;
        scan_pos = 0;
        frame_has_vcl = false;
        dropped_leading = false;
        stats = Stats{};
    }
};

// =============================================================================
//  EsSplitter
// =============================================================================

EsSplitter::EsSplitter(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

EsSplitter::~EsSplitter() = default;
EsSplitter::EsSplitter(EsSplitter&&) noexcept = default;
EsSplitter& EsSplitter::operator=(EsSplitter&&) noexcept = default;

std::unique_ptr<EsSplitter> EsSplitter::create(const Config& cfg, std::error_code& ec)
{
    ec.clear();
    if (cfg.max_frame_bytes == 0 || cfg.max_buffered_bytes == 0) {
        ec = make_error_code(MpegErrc::invalid_argument);
        return nullptr;
    }
    if (cfg.codec != Codec::h264 && cfg.codec != Codec::h265) {
        ec = make_error_code(MpegErrc::unsupported_codec);
        return nullptr;
    }
    return std::unique_ptr<EsSplitter>(new EsSplitter(std::make_unique<Impl>(cfg)));
}

std::error_code EsSplitter::feed(span<const std::uint8_t> data,
                                 const FrameCallback& on_frame)
{
    return impl_->feed(data, on_frame);
}

std::error_code EsSplitter::flush(const FrameCallback& on_frame)
{
    return impl_->flush(on_frame);
}

void EsSplitter::reset() noexcept
{
    impl_->reset();
}

const EsSplitter::Stats& EsSplitter::stats() const noexcept
{
    return impl_->stats;
}

} // namespace mpegmech
