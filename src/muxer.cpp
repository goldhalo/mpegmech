// =============================================================================
//  muxer.cpp —— 复用器（当前实现：PS）
// =============================================================================
#include <mpegmech/muxer.hpp>

#include "ps_muxer.hpp"

#include <algorithm>
#include <cstring>
#include <map>

namespace mpegmech {

namespace detail {

/// 视频码率未知时的默认值（bits/s）。PS 的 SystemHeader/PackHeader 要用。
/// 32 Mbps 足够覆盖 4K，对带宽估算没有实际影响（只是个声明值）。
constexpr std::uint32_t kDefaultMuxRate = 32u * 1000u * 1000u / 8u;

} // namespace detail

// =============================================================================
//  Impl
// =============================================================================

class Muxer::Impl {
public:
    Impl(const MuxerConfig& c, OutputCallback cb)
        : cfg(c), on_out(std::move(cb)) {}

    MuxerConfig    cfg;
    OutputCallback on_out;
    MuxStats       stats;

    struct StreamState {
        StreamConfig             cfg;
        int                      id = -1;
        std::uint8_t             stream_id = 0xE0;   ///< PS 用
        std::int64_t             last_pts = 0;
        bool                     has_pts = false;
        bool                     sent_header = false;
    };

    std::vector<StreamState> streams;
    detail::Bytes            buf;        ///< 组包缓冲（复用，减少分配）
    bool                     flushed = false;
    bool                     failed  = false;
    std::error_code          fail_ec;

    // ---- 输出 ----

    void emit()
    {
        if (buf.empty()) {
            return;
        }
        stats.bytes_out += buf.size();
        on_out(span<const std::uint8_t>(buf.data(), buf.size()));
        buf.clear();
    }

    std::error_code fail(std::error_code ec)
    {
        failed  = true;
        fail_ec = ec;
        return ec;
    }

    // ---- PS 专用 ----

    /// 输出"每个关键帧前都要带的"三件套。
    void emit_ps_headers()
    {
        std::vector<detail::SystemStreamEntry> sys;
        std::vector<detail::PsmStreamEntry>    psm;

        for (const auto& s : streams) {
            sys.push_back({s.stream_id, 32 * 1024});
            psm.push_back({stream_type_of(s.cfg.codec), s.stream_id});
        }

        detail::build_pack_header(buf, 0, detail::kDefaultMuxRate);
        if (cfg.ps_system_header) {
            detail::build_system_header(buf, detail::kDefaultMuxRate, sys);
        }
        detail::build_psm(buf, psm, 0);
        ++stats.psi_sent;
    }

    static std::uint8_t stream_type_of(Codec c) noexcept
    {
        switch (c) {
        case Codec::h264:  return 0x1B;
        case Codec::h265:  return 0x24;
        case Codec::h266:  return 0x33;
        case Codec::mpeg2_video: return 0x02;
        case Codec::mpeg4_video: return 0x10;
        case Codec::aac:   return 0x0F;
        case Codec::mp3:   return 0x03;
        case Codec::g711a: return 0x90;
        case Codec::g711u: return 0x91;
        default:           return 0x06;
        }
    }

    // ---- 对外 ----

    std::error_code write(int sid, FrameFlag flags, std::int64_t pts,
                          std::int64_t dts, span<const std::uint8_t> es)
    {
        if (failed) {
            return fail_ec;
        }
        if (flushed) {
            return make_error_code(MpegErrc::already_flushed);
        }
        if (sid < 0 || static_cast<std::size_t>(sid) >= streams.size()) {
            return make_error_code(MpegErrc::stream_not_found);
        }
        if (es.empty()) {
            return make_error_code(MpegErrc::invalid_argument);
        }

        auto& st = streams[static_cast<std::size_t>(sid)];

        // PTS 单调性检查
        if (st.has_pts && pts < st.last_pts && !cfg.allow_pts_rewind) {
            return make_error_code(MpegErrc::malformed);
        }

        // ★ 关键帧前必带三件套（GB28181 平台靠它定位帧边界）
        if ((flags & FrameFlag::key) != FrameFlag::none || !st.sent_header) {
            emit_ps_headers();
            st.sent_header = true;
        }

        // ★ PS 的 PES **必须填真实长度**。
        //
        //   早先我按"不定长（length=0）"实现，理由是可以流式封装不用缓冲
        //   整帧。但实测 ffmpeg 直接报 "could not find codec parameters" ——
        //   它不接受 PS 里的不定长 PES（不定长是 TS 的用法，那边靠 PUSI
        //   和 188 包边界分界，PS 没有这个机制）。
        detail::build_pes_header(buf, st.stream_id, pts, dts, es.size(),
                                 /*explicit_len=*/true);
        buf.insert(buf.end(), es.begin(), es.end());

        st.last_pts = pts;
        st.has_pts  = true;

        ++stats.frames_in;
        stats.bytes_in += es.size();
        ++stats.packets_out;               // PS: 一个 PES 算一次

        emit();
        return {};
    }

    std::error_code flush()
    {
        if (failed) {
            return fail_ec;
        }
        flushed = true;
        emit();
        return {};
    }

    std::error_code add_stream(const StreamConfig& sc, std::optional<int>& out_id)
    {
        if (failed) {
            return fail_ec;
        }
        if (sc.codec == Codec::unknown) {
            return make_error_code(MpegErrc::invalid_argument);
        }
        if (cfg.container != Container::ps) {
            return make_error_code(MpegErrc::unsupported_container);
        }

        StreamState st;
        st.cfg = sc;
        st.id  = static_cast<int>(streams.size());
        // PS 的 stream_id：视频 0xE0 起，音频 0xC0 起
        if (sc.stream_id != 0) {
            st.stream_id = sc.stream_id;
        } else {
            std::uint8_t base = isVideo(sc.codec) ? 0xE0 : 0xC0;
            for (const auto& s : streams) {
                if ((isVideo(s.cfg.codec) ? 0xE0 : 0xC0) == base &&
                    s.stream_id >= base) {
                    base = static_cast<std::uint8_t>(s.stream_id + 1);
                }
            }
            st.stream_id = base;
        }

        streams.push_back(std::move(st));
        out_id = streams.back().id;
        return {};
    }
};

// =============================================================================
//  Muxer
// =============================================================================

Muxer::Muxer(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
Muxer::~Muxer() = default;
Muxer::Muxer(Muxer&&) noexcept = default;
Muxer& Muxer::operator=(Muxer&&) noexcept = default;

std::unique_ptr<Muxer> Muxer::create(const MuxerConfig& cfg, OutputCallback on_out,
                                     std::error_code& ec)
{
    ec.clear();
    if (!on_out) {
        ec = make_error_code(MpegErrc::invalid_argument);
        return nullptr;
    }
    if (cfg.container == Container::unknown) {
        ec = make_error_code(MpegErrc::unsupported_container);
        return nullptr;
    }
    return std::unique_ptr<Muxer>(new Muxer(std::make_unique<Impl>(cfg, std::move(on_out))));
}

std::optional<int> Muxer::addStream(const StreamConfig& cfg, std::error_code& ec)
{
    std::optional<int> id;
    ec = impl_->add_stream(cfg, id);
    return ec ? std::nullopt : id;
}

std::error_code Muxer::write(int stream_id, FrameFlag flags, std::int64_t pts,
                             std::int64_t dts, span<const std::uint8_t> es)
{
    return impl_->write(stream_id, flags, pts, dts, es);
}

std::error_code Muxer::flush()
{
    return impl_->flush();
}

const MuxStats& Muxer::stats() const noexcept
{
    return impl_->stats;
}

std::size_t Muxer::streamCount() const noexcept
{
    return impl_->streams.size();
}

} // namespace mpegmech
