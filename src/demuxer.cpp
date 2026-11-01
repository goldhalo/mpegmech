// =============================================================================
//  demuxer.cpp —— 解封装（当前实现：PS）
// =============================================================================
//  PS 是**起始码驱动**的，所以能自同步：
//
//    [PackHeader BA][SystemHeader BB][PSM BC][PES E0][PES E0]...
//
//  从流中间接入时，扫到第一个 `00 00 01 BA` 就开始工作 —— 这就是为什么
//  PS 允许"非开头输入"，而 TS 必须等 PAT/PMT。
// =============================================================================
#include <mpegmech/demuxer.hpp>

#include <algorithm>
#include <cstring>
#include <map>
#include <vector>

namespace mpegmech {

namespace {

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

std::uint16_t get16(const std::uint8_t* p) noexcept
{
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

/// 解 5 字节 PTS/DTS 编码。
std::int64_t get_timestamp(const std::uint8_t* p) noexcept
{
    return (static_cast<std::int64_t>(p[0] & 0x0E) << 29) |
           (static_cast<std::int64_t>(p[1]) << 22) |
           (static_cast<std::int64_t>(p[2] & 0xFE) << 14) |
           (static_cast<std::int64_t>(p[3]) << 7) |
           (static_cast<std::int64_t>(p[4] & 0xFE) >> 1);
}

/// PS 的 stream_type（PSM 里的）→ Codec。
Codec codec_of(std::uint8_t st) noexcept
{
    switch (st) {
    case 0x1B: return Codec::h264;
    case 0x24: return Codec::h265;
    case 0x33: return Codec::h266;
    case 0x02: return Codec::mpeg2_video;
    case 0x10: return Codec::mpeg4_video;
    case 0x0F: return Codec::aac;
    case 0x03:
    case 0x04: return Codec::mp3;
    case 0x90: return Codec::g711a;
    case 0x91: return Codec::g711u;
    default:   return Codec::unknown;
    }
}

/// 按 stream_id 猜 codec（PSM 缺失时的兜底）。
Codec codec_by_stream_id(std::uint8_t sid) noexcept
{
    if (sid >= 0xE0 && sid <= 0xEF) { return Codec::h264; }   // 视频，默认 H264
    if (sid >= 0xC0 && sid <= 0xDF) { return Codec::aac;  }   // 音频，默认 AAC
    return Codec::unknown;
}

} // namespace

// =============================================================================
//  Impl
// =============================================================================

class Demuxer::Impl {
public:
    Impl(const DemuxerConfig& c, FrameCallback f, StreamCallback s)
        : cfg(c), on_frame(std::move(f)), on_stream(std::move(s)) {}

    DemuxerConfig  cfg;
    FrameCallback  on_frame;
    StreamCallback on_stream;

    DemuxStats stats;

    enum class State { searching, synced };
    State state = State::searching;

    std::vector<std::uint8_t> in;
    std::size_t               scan = 0;      ///< 扫描位置

    /// PS 的 elementary_stream_id → 库内 stream_id
    std::map<std::uint8_t, int> streams;
    /// 库内 stream_id → codec
    std::map<int, Codec>        codecs;
    int                         next_stream_id = 0;

    bool failed = false;
    std::error_code fail_ec;

    // ---- 辅助 ----

    void compact()
    {
        if (scan == 0) { return; }
        if (scan >= in.size()) {
            in.clear();
            scan = 0;
            return;
        }
        in.erase(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(scan));
        scan = 0;
    }

    /// 确保某个 PS 流有对应的库内 id，必要时触发 stream 回调。
    int ensure_stream(std::uint8_t ps_sid, Codec codec)
    {
        auto it = streams.find(ps_sid);
        if (it != streams.end()) {
            // codec 后来才知道（PSM 晚到）时更新
            if (codec != Codec::unknown && codecs[it->second] == Codec::unknown) {
                codecs[it->second] = codec;
                if (on_stream) {
                    StreamInfo si;
                    si.stream_id = it->second;
                    si.codec     = codec;
                    si.program   = 1;
                    on_stream(si);
                }
            }
            return it->second;
        }

        const int id = next_stream_id++;
        streams[ps_sid] = id;
        codecs[id]      = codec;

        if (on_stream) {
            StreamInfo si;
            si.stream_id = id;
            si.codec     = codec;
            si.program   = 1;
            si.pid       = 0;
            si.adts      = true;
            on_stream(si);
        }
        return id;
    }

    void emit_frame(std::uint8_t ps_sid, const std::uint8_t* payload, std::size_t len,
                    std::int64_t pts, std::int64_t dts, bool incomplete)
    {
        if (len == 0) { return; }
        if (len > cfg.max_frame_bytes) {
            ++stats.parse_errors;
            return;
        }

        const int   id    = ensure_stream(ps_sid, codec_by_stream_id(ps_sid));
        const Codec codec = codecs[id];

        FrameInfo fi;
        fi.stream_id = id;
        fi.codec     = codec;
        fi.pts       = pts;
        fi.dts       = (dts == kNoTimestamp) ? pts : dts;
        fi.program   = 1;
        fi.flags     = FrameFlag::none;
        if (incomplete) {
            fi.flags = fi.flags | FrameFlag::incomplete;
            ++stats.incomplete_frames;
        }
        // 关键帧判断：H.264/H.265 看有没有 IDR（扫描前几个 NAL）
        if (isVideo(codec)) {
            for (std::size_t i = 0; i + 5 < len && i < 256; ++i) {
                if (payload[i] == 0 && payload[i+1] == 0 && payload[i+2] == 1) {
                    if (codec == Codec::h264) {
                        const std::uint8_t t = payload[i+3] & 0x1F;
                        if (t == 5) { fi.flags = fi.flags | FrameFlag::key; break; }
                    } else if (codec == Codec::h265) {
                        const std::uint8_t t = (payload[i+3] >> 1) & 0x3F;
                        if (t >= 16 && t <= 21) { fi.flags = fi.flags | FrameFlag::key; break; }
                    }
                }
            }
        }

        ++stats.frames_out;
        on_frame(fi, span<const std::uint8_t>(payload, len));
    }

    // ---- 主循环 ----

    std::error_code feed(span<const std::uint8_t> data)
    {
        if (failed) { return fail_ec; }

        if (!data.empty()) {
            in.insert(in.end(), data.begin(), data.end());
            stats.bytes_in += data.size();
        }

        if (in.size() - scan > cfg.max_buffered_bytes) {
            ++stats.overflow_drops;
            const std::size_t keep = cfg.max_buffered_bytes / 2;
            const std::size_t drop = in.size() - keep;
            stats.bytes_skipped += drop;
            in.erase(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(drop));
            scan  = 0;
            state = State::searching;
            return make_error_code(MpegErrc::buffer_overflow);
        }

        return parse();
    }

    std::error_code parse()
    {
        while (true) {
            const std::size_t avail = in.size() - scan;
            if (avail < 4) { break; }

            const std::size_t sc = startcode_len(in.data() + scan, avail);
            if (sc == 0) {
                if (state == State::searching) {
                    ++scan;
                    ++stats.bytes_skipped;
                } else {
                    ++scan;
                }
                continue;
            }

            // startcode 长度是 3（00 00 01）或 4（00 00 00 01），
            // 功能码（BA/BB/BC/E0...）紧跟在它**之后**，即 scan + sc。
            const std::size_t code_at = scan + sc;
            const std::uint8_t code = in[code_at];

            // ---- PackHeader (0xBA)：14 字节固定 ----
            if (code == 0xBA) {
                if (state == State::searching) { state = State::synced; }
                if (avail < sc + 14) { break; }          // 等够
                scan = code_at + 15;                     // 跳过整个 pack header
                continue;
            }

            if (state == State::searching) {
                ++scan;
                ++stats.bytes_skipped;
                continue;
            }

            // ---- SystemHeader (0xBB) / PSM (0xBC)：带长度 ----
            if (code == 0xBB || code == 0xBC || code == 0xBE) {
                if (avail < sc + 3) { break; }
                const std::size_t body = get16(in.data() + code_at + 1);
                if (avail < sc + 3 + body) { break; }
                if (code == 0xBC) {
                    parse_psm(code_at + 3, body);
                }
                scan = code_at + 3 + body;
                continue;
            }

            // ---- PES (0xBD / 0xC0-0xDF / 0xE0-0xEF) ----
            const bool is_pes = (code == 0xBD) ||
                                (code >= 0xC0 && code <= 0xDF) ||
                                (code >= 0xE0 && code <= 0xEF);
            if (!is_pes) {
                ++scan;
                ++stats.parse_errors;
                continue;
            }

            if (avail < sc + 3) { break; }
            const std::size_t pes_len = get16(in.data() + code_at + 1);

            // PES 头：flags(2) + header_data_length(1) + 扩展
            if (avail < sc + 3 + 3) { break; }
            const std::uint8_t hdr_len = in[code_at + 3 + 2];
            const std::size_t  hdr_end = code_at + 3 + 3 + hdr_len;

            // 帧尾位置。
            //
            // PES_packet_length 统计的是"长度字段之后"的字节数，即从
            // `code_at + 3`（长度字段 2 字节之后）算起：
            //     [起始码×sc][E0][len 2][flags 2][hdr_len 1][PTS][payload]
            //                ^code_at  ^^^^^^^^^^^^^ pes_len 覆盖这一段
            std::size_t frame_end = 0;
            bool        need_more = false;
            if (pes_len > 0) {
                frame_end = code_at + 3 + pes_len;
                if (code_at + 3 + pes_len > in.size()) { need_more = true; }
            } else {
                // 不定长：到下一个起始码为止
                std::size_t p = hdr_end;
                while (p + 3 <= in.size()) {
                    if (startcode_len(in.data() + p, in.size() - p) > 0) { break; }
                    ++p;
                }
                if (p + 3 > in.size()) { need_more = true; }
                frame_end = p;
            }
            if (need_more) { break; }

            // 提取 PTS/DTS
            //
            //   [E0][len 2][flags 2][hdr_len 1][PTS 5][DTS 5]
            //   code_at  +1,+2  +3,+4    +5     +6     +11
            //
            //   ★ PTS_DTS_flags 在 flags 的第 1 字节的最高 2 位（code_at + 3），
            //     不是 PTS 的首字节（那个高位是 '0010' 前缀）。
            std::int64_t pts = kNoTimestamp;
            std::int64_t dts = kNoTimestamp;
            if (hdr_len >= 5 && code_at + 6 + 5 <= in.size()) {
                const std::uint8_t pes_flags = in[code_at + 3];
                const int          pts_dts   = (pes_flags >> 6) & 0x03;
                const std::uint8_t* h = in.data() + code_at + 6;
                if (pts_dts == 0x02 || pts_dts == 0x03) {
                    pts = get_timestamp(h);
                    if (pts_dts == 0x03 && hdr_len >= 10) {
                        dts = get_timestamp(h + 5);
                    }
                }
            }

            const std::size_t payload_at  = hdr_end;
            const std::size_t payload_end = frame_end;
            if (payload_end <= payload_at || payload_end > in.size()) {
                scan = code_at + 1;
                ++stats.parse_errors;
                continue;
            }

            emit_frame(code, in.data() + payload_at, payload_end - payload_at,
                       pts, dts, false);

            scan = payload_end;
        }

        compact();
        return {};
    }

    void parse_psm(std::size_t at, std::size_t len)
    {
        // PSM 结构：current_next(1) + ver(5) + reserved(7) + marker(1)
        //           program_stream_info_length(2) + info
        //           elementary_stream_map_length(2) + entries
        if (len < 4) { return; }
        const std::size_t info_len = get16(in.data() + at + 2);
        std::size_t p = at + 4 + info_len;
        if (p + 2 > at + len) { return; }
        const std::size_t map_len = get16(in.data() + p);
        p += 2;

        std::size_t remaining = map_len;
        while (remaining >= 4 && p + 4 <= at + len) {
            const std::uint8_t st = in[p];
            const std::uint8_t sid = in[p + 1];
            const std::size_t  ilen = get16(in.data() + p + 2);
            const Codec c = codec_of(st);
            if (c != Codec::unknown) {
                ensure_stream(sid, c);
            }
            p += 4 + ilen;
            if (remaining < 4 + ilen) { break; }
            remaining -= 4 + ilen;
        }
    }

    std::error_code flush()
    {
        // 不吐残缺帧（emit_partial_on_flush=false 时的行为）
        if (cfg.emit_partial_on_flush) {
            // 剩余未解析的当作一帧
            if (scan < in.size()) {
                emit_frame(0xE0, in.data() + scan, in.size() - scan,
                           kNoTimestamp, kNoTimestamp, true);
            }
        }
        in.clear();
        scan  = 0;
        state = State::searching;
        return {};
    }
};

// =============================================================================
//  Demuxer
// =============================================================================

Demuxer::Demuxer(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
Demuxer::~Demuxer() = default;
Demuxer::Demuxer(Demuxer&&) noexcept = default;
Demuxer& Demuxer::operator=(Demuxer&&) noexcept = default;

std::unique_ptr<Demuxer> Demuxer::create(const DemuxerConfig& cfg,
                                         FrameCallback on_frame,
                                         StreamCallback on_stream,
                                         std::error_code& ec)
{
    ec.clear();
    if (!on_frame) {
        ec = make_error_code(MpegErrc::invalid_argument);
        return nullptr;
    }
    return std::unique_ptr<Demuxer>(
        new Demuxer(std::make_unique<Impl>(cfg, std::move(on_frame), std::move(on_stream))));
}

std::error_code Demuxer::feed(span<const std::uint8_t> data)
{
    return impl_->feed(data);
}

std::error_code Demuxer::flush()
{
    return impl_->flush();
}

void Demuxer::setHandler(FrameCallback on_frame)
{
    impl_->on_frame = std::move(on_frame);
}

void Demuxer::reset() noexcept
{
    impl_->in.clear();
    impl_->scan  = 0;
    impl_->state = Impl::State::searching;
    impl_->streams.clear();
    impl_->codecs.clear();
    impl_->next_stream_id = 0;
    impl_->stats = DemuxStats{};
    impl_->failed = false;
    impl_->fail_ec.clear();
}

bool Demuxer::failed() const noexcept
{
    return impl_->failed;
}

const DemuxStats& Demuxer::stats() const noexcept
{
    return impl_->stats;
}

} // namespace mpegmech
