// =============================================================================
//  demuxer.cpp —— 解封装（PS + TS）
// =============================================================================
//  PS 是**起始码驱动**的，所以能自同步：
//
//    [PackHeader BA][SystemHeader BB][PSM BC][PES E0][PES E0]...
//
//  从流中间接入时，扫到第一个 `00 00 01 BA` 就开始工作 —— 这就是为什么
//  PS 允许"非开头输入"，而 TS 必须等 PAT/PMT。
//
//  TS 是**表驱动**的：188 字节包本身不带"我是什么"的信息，PID 0 的 PAT
//  告诉你 PMT 在哪个 PID，PMT 再告诉你哪个 PID 是视频/音频。所以从流中间
//  接入要等一个 PSI 周期（通常 100~500ms），这期间的数据只能丢。
//  两条路径的公共部分只有"喂任意长度输入"和"回调帧/流"，其余完全分开，
//  因此放在同一个 Impl 里按 `detected` 分叉，不强行抽象。
// =============================================================================
#include <mpegmech/demuxer.hpp>

#include <algorithm>
#include <cstring>
#include <map>
#include <vector>
#include <memory>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <utility>

#include "ts_demuxer.hpp"
#include "ts_muxer.hpp"   // crc32_mpeg2

namespace mpegmech {

namespace {

/// 自动嗅探需要的字节数：TS 要连续 3 个 0x47（间隔 188），至少 564 字节。
constexpr std::size_t kSniffBytes = 1024;

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

std::uint32_t get32(const std::uint8_t* p) noexcept
{
    return (static_cast<std::uint32_t>(p[0]) << 24) |
           (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) |
           static_cast<std::uint32_t>(p[3]);
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

/// PSM 缺失时**不再**按 stream_id 猜 codec。
///
/// 以前把 0xE0..0xEF 猜成 H.264、0xC0..0xDF 猜成 AAC —— 但 GB28181 里
/// 0xC0 起的常是 G.711，被猜成 AAC 后下游解码必错。PS 的 codec 只认 PSM，
/// 缺 PSM 就返回 unknown，让调用方自己决定（诚实优于瞎猜）。
Codec codec_by_stream_id(std::uint8_t sid) noexcept
{
    (void)sid;
    return Codec::unknown;
}

} // namespace

/// ADTS 的 sampling_frequency_index → 采样率（索引 13/14 是保留值，15 是显式）。
constexpr int kAdtsSamplingRate[13] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000,
    22050, 16000, 12000, 11025,  8000,  7350,
};

/// 视频帧是否关键帧（只看前 256 字节里的 NAL 头）。
///
/// @note 只用于**填标志位**，不参与解码判断。看不到 VCL 时保守返回 false，
///       宁可少标也不要把非关键帧标成关键帧（下游可能据此丢帧）。
bool looks_key(Codec codec, const std::uint8_t* p, std::size_t len) noexcept
{
    if (!isVideo(codec)) { return false; }
    for (std::size_t i = 0; i + 5 < len && i < 256; ++i) {
        if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1) {
            if (codec == Codec::h264) {
                if ((p[i + 3] & 0x1F) == 5) { return true; }        // IDR
            } else if (codec == Codec::h265) {
                const std::uint8_t t = (p[i + 3] >> 1) & 0x3F;
                if (t >= 16 && t <= 21) { return true; }            // BLA..CRA
            } else if (codec == Codec::h266) {
                // H.266 的 NAL header 是 2 字节：type 占 bits 1..5
                // （与 H.265 同位），IDR_W_RADL=7 / IDR_N_LP=8 / CRA=9 /
                // GDR=10 / RSV_IRAP_11=11 —— 都是"可以起播"的帧。
                if (i + 4 < len) {
                    const std::uint8_t t = static_cast<std::uint8_t>((p[i + 4] >> 3) & 0x1F);
                    if (t >= 7 && t <= 11) { return true; }
                }
            }
        }
    }
    return false;
}

// =============================================================================
//  Impl
// =============================================================================

struct Demuxer::Impl {
public:
    Impl(const DemuxerConfig& c, FrameCallback f, StreamCallback s)
        : cfg(c), on_frame(std::move(f)), on_stream(std::move(s)) {}

    DemuxerConfig  cfg;
    FrameCallback  on_frame;
    StreamCallback on_stream;

    DemuxStats stats;

    /// 已确定的容器（`unknown` = 还没嗅探出来）。
    Container detected = Container::unknown;

    // ---- TS 专用状态 ----
    detail::TsDemuxer            ts;
    std::map<std::uint16_t, int> ts_pid_to_id;   ///< TS PID → 库内 stream_id
    std::uint64_t                ts_frame_errors = 0;  ///< TS 路径下的帧错误数（超长丢弃、AAC 尾部无法成帧等）

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
        // setHandler(nullptr) 会解绑回调；此时 feed 仍要消费数据（契约见
        // demuxer.hpp 的 setHandler），只是不再回调 —— 空 std::function
        // 直接调用会抛 std::bad_function_call 并炸穿 feed()。
        if (!on_frame) { return; }

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

    // ---- TS 路径 ----

    /// 确保 TS 的某个 ES 有库内 stream_id，必要时触发 stream 回调。
    int ensure_ts_stream(const detail::TsEsInfo& e)
    {
        auto it = ts_pid_to_id.find(e.pid);
        if (it != ts_pid_to_id.end()) {
            // PMT 更新后 stream_type 可能变（罕见，但流切换时会遇到）
            const Codec c = codec_of(e.stream_type);
            if (c != Codec::unknown && codecs[it->second] == Codec::unknown) {
                codecs[it->second] = c;
                if (on_stream) {
                    StreamInfo si;
                    si.stream_id = it->second;
                    si.codec     = c;
                    si.program   = e.program;
                    si.pid       = e.pid;
                    on_stream(si);
                }
            }
            return it->second;
        }

        const int id = next_stream_id++;
        ts_pid_to_id[e.pid] = id;
        codecs[id]          = codec_of(e.stream_type);

        if (on_stream) {
            StreamInfo si;
            si.stream_id = id;
            si.codec     = codecs[id];
            si.program   = e.program;
            si.pid       = e.pid;
            si.adts      = true;    // TS 里的 AAC 恒带 ADTS 头
            on_stream(si);
        }
        return id;
    }

    /// 发出一个 TS 帧（统一填 FrameInfo、计统计）。
    void push_ts_frame(int id, Codec codec, std::uint16_t program,
                       std::int64_t pts, std::int64_t dts,
                       const std::uint8_t* p, std::size_t n,
                       bool incomplete = false)
    {
        if (n == 0) { return; }
        if (!on_frame) { return; }   // setHandler(nullptr) 之后不再回调

        FrameInfo fi;
        fi.stream_id = id;
        fi.codec     = codec;
        fi.pts       = pts;
        // 与 PS 路径（emit_frame）统一：没有 DTS 时按 pts 处理，
        // 否则 FrameInfo.dts 会漏出 kNoTimestamp 哨兵值。
        fi.dts       = (dts == kNoTimestamp) ? pts : dts;
        fi.program   = program;
        if (isAudio(codec)) {
            fi.flags = fi.flags | FrameFlag::key;      // 音频帧恒可作为起点
        } else if (looks_key(codec, p, n)) {
            fi.flags = fi.flags | FrameFlag::key;
        }
        if (incomplete) {
            fi.flags = fi.flags | FrameFlag::incomplete;
            ++stats.incomplete_frames;
        }

        ++stats.frames_out;
        on_frame(fi, span<const std::uint8_t>(p, n));
    }

    /// 把 TS 解出的帧派发出去。
    std::error_code dispatch_ts(std::vector<detail::TsDemuxer::Frame>& frames,
                                bool incomplete = false)
    {
        // 先把流表同步到最新。PMT 一定先于该 PID 的 PES 被解析，所以在这里
        // 同步能保证"stream 回调早于该流的第一帧"这条契约。
        for (const auto& e : ts.streams()) {
            ensure_ts_stream(e);
        }

        for (auto& f : frames) {
            if (f.payload.empty()) { continue; }

            auto it = ts_pid_to_id.find(f.pid);
            if (it == ts_pid_to_id.end()) { continue; }   // PMT 里没有的 PID

            const int   id    = it->second;
            const Codec codec = codecs[id];

            // ---- AAC：一条 PES 里可能塞了多个 ADTS 帧 ----
            //
            //  我们自己的复用器是"一帧一 PES"，但 ffmpeg / 摄像机 / 平台侧
            //  常常把几十个 AAC 帧攒成一条 PES（省包头）。若按 PES 边界整段
            //  交出去，调用方拿到的是"一坨 ADTS"，播放器只能当一帧用。
            //  这里按 ADTS 的 frame_length 拆开，PTS 按 1024 样本递增。
            if (codec == Codec::aac) {
                const std::size_t total = f.payload.size();
                std::size_t i = 0;
                std::int64_t pts = f.pts;
                std::int64_t step = 0;

                while (i + 7 <= total) {
                    const std::uint8_t* h = f.payload.data() + i;
                    if (h[0] != 0xFF || (h[1] & 0xF0) != 0xF0) { break; }
                    const std::size_t flen =
                        (static_cast<std::size_t>(h[3] & 0x03) << 11) |
                        (static_cast<std::size_t>(h[4]) << 3) |
                        ((h[5] >> 5) & 0x07);
                    const std::size_t hdr = (h[1] & 0x01) ? 7u : 9u;
                    if (flen < hdr || i + flen > total) { break; }

                    if (step == 0) {
                        const int idx = (h[2] >> 2) & 0x0F;
                        const int rate = (idx < 13) ? kAdtsSamplingRate[idx] : 0;
                        step = (rate > 0)
                                   ? static_cast<std::int64_t>(1024) * kClockHz / rate
                                   : 0;
                    }
                    if (flen > cfg.max_frame_bytes) { ++ts_frame_errors; }
                    else { push_ts_frame(id, codec, f.program, pts, pts, h, flen,
                                         incomplete); }

                    if (pts != kNoTimestamp && step > 0) { pts += step; }
                    i += flen;
                }
                // 循环因非 ADTS 数据提前 break：剩余尾部成不了帧，计错误，
                // 免得静默丢弃（以前完全无痕）。
                if (i != 0 && i < total) { ++ts_frame_errors; }
                if (i == 0) {   // 不是 ADTS：原样透传，别把数据吞掉
                    if (total > cfg.max_frame_bytes) { ++ts_frame_errors; }
                    else { push_ts_frame(id, codec, f.program, f.pts, f.dts,
                                         f.payload.data(), total, incomplete); }
                }
                continue;
            }

            if (f.payload.size() > cfg.max_frame_bytes) {
                ++ts_frame_errors;
                continue;
            }
            push_ts_frame(id, codec, f.program, f.pts, f.dts,
                          f.payload.data(), f.payload.size(), incomplete);
        }

        stats.bytes_skipped = ts.bytes_skipped();
        stats.resyncs       = ts.resyncs();
        stats.parse_errors  = ts.bad_packets() + ts_frame_errors;

        // ★ 判据必须是"**本次**失步以来跳过了多少"，不是 lifetime 累计。
        //
        //   以前用 bytes_skipped()（只在 reset() 清零）：长播流上它只会单调上涨，
        //   过阈值之后**每一次一个字节的网络毛刺都会立刻返回 sync_lost** ——
        //   把"抖动容忍"的防御机制变成了"长播流必炸"。调用方按契约会认为
        //   整条流已放弃（需要 reset()），于是一路还活着的流被丢掉。
        if (cfg.ts_max_resync_bytes != 0 &&
            ts.state() == detail::TsSyncState::searching &&
            ts.resync_skipped() > cfg.ts_max_resync_bytes) {
            return make_error_code(MpegErrc::sync_lost);
        }
        return {};
    }

    std::error_code feed_ts(const std::uint8_t* p, std::size_t n)
    {
        std::vector<detail::TsDemuxer::Frame> frames;
        ts.feed(p, n, frames);
        auto ec = dispatch_ts(frames);
        if (ec) {
            return ec;
        }

        // ★ 缓冲上限对 TS 同样成立。
        //
        //   以前 `max_buffered_bytes` 只在 PS 路径检查，TS 的每-PID PES 累积是
        //   **无界**的：`pes_len == 0`（视频标配的不定长 PES）在等到下一条 PUSI
        //   之前会一直涨。从流中间接入、PID 长期不给 PUSI（受损流/流切换残留）、
        //   恶意构造的"PUSI + 无界尾巴"都能把内存撑爆。
        //   而 demuxer.hpp 对两条路径承诺的是同一套语义（丢最旧 + buffer_overflow）。
        if (cfg.max_buffered_bytes != 0 &&
            ts.buffered_bytes() > cfg.max_buffered_bytes) {
            ++stats.overflow_drops;
            stats.bytes_skipped += ts.buffered_bytes();
            ts.discard_partial();     // PES 只能整条丢（半个 PES 没有意义）
            return make_error_code(MpegErrc::buffer_overflow);
        }
        return {};
    }

    // ---- 主循环 ----

    std::error_code feed(span<const std::uint8_t> data)
    {
        if (failed) { return fail_ec; }

        if (!data.empty()) {
            stats.bytes_in += data.size();
        }

        // ---- 1. 确定容器 ----
        if (detected == Container::unknown && cfg.container != Container::unknown) {
            detected = cfg.container;
        }

        if (detected == Container::unknown) {
            // 攒够嗅探窗口再判。PS 的判据只要 4 字节（00 00 01 BA），
            // 所以短流也能一次定下来；TS 要 564 字节，不足就继续等。
            // 攒到 kSniffBytes 还认不出来，就按 PS 兜底（PS 的 PackHeader
            // 一定在起始处，认不出来通常意味着调用方给的压根不是 PS/TS，
            // 这时按 PS 试一遍更容易在统计里看到痕迹）。
            if (!data.empty()) {
                in.insert(in.end(), data.begin(), data.end());
            }

            detected = sniffContainer(span<const std::uint8_t>(in.data(), in.size()));
            if (detected == Container::unknown) {
                if (in.size() < kSniffBytes) { return {}; }
                // ★ 攒够了还认不出来 = 既不是 PS 也不是 TS。
                //
                //   以前这里强制按 PS 处理，于是 `container()` 谎报成 ps，
                //   错误里永远只有 bytes_skipped 慢慢涨 —— 调用方完全不知道
                //   自己喂进来的根本不是这两种容器（`unsupported_container`
                //   这个错误码对 Demuxer 从不返回）。
                //
                // ★ 清空嗅探窗口：否则调用方继续 feed 时，每次都会把新数据
                //   追加进 in 再返回同一个错误，缓冲随喂入量无界增长。
                in.clear();
                scan = 0;
                return make_error_code(MpegErrc::unsupported_container);
            }

            std::vector<std::uint8_t> pending;
            pending.swap(in);
            scan = 0;
            if (detected == Container::ts) {
                return feed_ts(pending.data(), pending.size());
            }
            return feed_ps(pending);
        }

        if (detected == Container::ts) {
            return feed_ts(data.data(), data.size());
        }

        return feed_ps(data);
    }

    // ---- PS 路径 ----

    std::error_code feed_ps(span<const std::uint8_t> data)
    {
        if (!data.empty()) {
            in.insert(in.end(), data.begin(), data.end());
        }

        // 0 = 不限制（与 ts_max_resync_bytes 的 0 语义一致）
        if (cfg.max_buffered_bytes != 0 && in.size() - scan > cfg.max_buffered_bytes) {
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
            // 防御：任何分支把 scan 推过末尾，`in.size() - scan` 会**无符号下溢**
            // 成一个天文数字，avail 判据随之失效，循环空转到超时。
            if (scan > in.size()) { scan = in.size(); break; }

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

            // start code 正好顶到缓冲末尾时（avail == sc），功能码还没进来。
            // 少了这一句，下一行的 in[code_at] 就是越界读：
            //   · Release / libstdc++ 下读到的"垃圾"有时恰好不是已知功能码，
            //     被跳过、看不出问题；
            //   · MSVC Debug 的迭代器检查直接 abort（exit code 3），
            //     Debug 版 test_g711_ps 之前卡 120 秒超时就是这条链引起。
            if (code_at >= in.size()) { break; }

            const std::uint8_t code = in[code_at];

            // ---- PackHeader (0xBA)：14 字节固定 ----
            if (code == 0xBA) {
                if (state == State::searching) { state = State::synced; }
                // PackHeader 总长 14 字节（4 字节起始码 + 10 字节字段），
                // code_at 指向 BA，所以从 scan 起的跳转量是 14 = sc + 11。
                //
                // 以前写成 sc + 15 / code_at + 15（比正确值多 4 字节）：
                // 紧随 PackHeader 的 PES 会被跳过 4 字节，整帧静默丢失。
                // 本库自己产生的流因为后面跟着 SystemHeader + PSM，
                // 多跳的 4 字节落进 SystemHeader 字段区，才"碰巧"没事。
                if (avail < sc + 11) { break; }
                scan = code_at + 11;                     // 跳过整个 pack header（14 字节）
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
            //   ★ PES 头有两个 flags 字节：
            //       flags1 = code_at + 3（'10' + 加扰位，标准值 0x80）
            //       flags2 = code_at + 4（PTS_DTS_flags 在最高 2 位）
            //     以前读 code_at + 3，拿到 0x80 → pts_dts 恒为 2（"只有 PTS"），
            //     DTS 永远解析不到、fi.dts 恒等于 pts（B 帧重排信息全丢）。
            std::int64_t pts = kNoTimestamp;
            std::int64_t dts = kNoTimestamp;
            if (hdr_len >= 5 && code_at + 6 + 5 <= in.size()) {
                const std::uint8_t pes_flags2 = in[code_at + 4];
                const int          pts_dts    = (pes_flags2 >> 6) & 0x03;
                const std::uint8_t* h = in.data() + code_at + 6;
                if (pts_dts == 0x02 || pts_dts == 0x03) {
                    pts = get_timestamp(h);
                    // DTS 在 h + 5，连读 5 字节；必须保证 hdr_len 与缓冲都够，
                    // 否则畸形 PES（hdr_len=10 但 pes_len 很小、缓冲恰好截断）
                    // 会越界读 4 字节。
                    if (pts_dts == 0x03 && hdr_len >= 10 &&
                        code_at + 6 + 10 <= in.size()) {
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

        // CRC32 校验：坏 PSM 直接丢弃 —— G.711 这类只能靠 PSM 识别的流
        // 若被损坏的映射表误导，会解出错误的 codec。
        if (detail::crc32_mpeg2(in.data() + at, len - 4) != get32(in.data() + at + len - 4)) {
            ++stats.parse_errors;
            return;
        }

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
        if (detected == Container::ts) {
            // ★ 与 PS 分支保持同一套语义。
            //
            //   以前这里无条件把残留 PES 交出去，还把这条写成"对 TS 不做区分"
            //   —— 但默认契约是 `emit_partial_on_flush = false`：
            //   "截断的帧对下游没有价值，且可能被误当完整帧解码"。
            //   断流/掉线时残留 PES 是常态，下游会把半条垃圾当完整帧送解码器，
            //   表现就是末帧花屏或者一条损坏帧的证据。
            //
            //   而且 TS 的残帧**没有** incomplete 标志可打（FrameInfo 里没这个字段），
            //   所以"交给调用方自己判"这条路本来也走不通。
            if (!cfg.emit_partial_on_flush) {
                ts.discard_partial();
                in.clear();
                scan  = 0;
                state = State::searching;
                return {};
            }
            std::vector<detail::TsDemuxer::Frame> frames;
            ts.flush(frames);
            // flush 交出的必然是残帧：标 incomplete，与 PS 路径一致。
            return dispatch_ts(frames, /*incomplete=*/true);
        }

        // 嗅探都没完成就 flush：残留是原始垃圾，不是某条 ES 的残帧，直接丢。
        if (detected == Container::unknown) {
            in.clear();
            scan  = 0;
            state = State::searching;
            return {};
        }

        // 不吐残缺帧（emit_partial_on_flush=false 时的行为）
        if (cfg.emit_partial_on_flush && scan < in.size()) {
            // 残帧按"已知的第一条流"标注：硬编码 0xE0 会把纯音频流错标成
            // 视频（codec 也会被兜底猜成 H.264）。
            const std::uint8_t ps_sid = streams.empty() ? 0xE0 : streams.begin()->first;
            emit_frame(ps_sid, in.data() + scan, in.size() - scan,
                       kNoTimestamp, kNoTimestamp, true);
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
    impl_->ts.reset();
    impl_->ts_pid_to_id.clear();
    impl_->ts_frame_errors = 0;
    impl_->detected = impl_->cfg.container;   // 明确指定的保留，自动嗅探的重来
    impl_->stats = DemuxStats{};
    impl_->failed = false;
    impl_->fail_ec.clear();
}

Container Demuxer::container() const noexcept
{
    return impl_->detected;
}

std::size_t Demuxer::buffered() const noexcept
{
    if (impl_->detected == Container::ts) {
        return impl_->ts.buffered();
    }
    return impl_->in.size() - impl_->scan;
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
