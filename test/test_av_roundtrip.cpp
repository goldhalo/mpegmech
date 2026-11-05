// =============================================================================
//  test_av_roundtrip.cpp —— H264 + AAC → PS → 解回
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <utility>

using namespace mpegmech;

namespace {
int g_fail = 0;
#define EXPECT(c, m) do { if (!(c)) { std::printf("  [FAIL] %s (line %d)\n", (m), __LINE__); ++g_fail; } } while (0)

std::vector<std::uint8_t> read_file(const char* p)
{
    std::vector<std::uint8_t> out;
    std::FILE* f = std::fopen(p, "rb");
    if (!f) return out;
    std::uint8_t b[65536]; std::size_t n;
    while ((n = std::fread(b, 1, sizeof(b), f)) > 0) out.insert(out.end(), b, b + n);
    std::fclose(f);
    return out;
}

std::vector<std::vector<std::uint8_t>> strip_adts(const std::vector<std::uint8_t>& raw)
{
    std::vector<std::vector<std::uint8_t>> frames;
    std::size_t p = 0;
    while (p + 7 <= raw.size()) {
        if (raw[p] != 0xFF || (raw[p+1] & 0xF0) != 0xF0) break;
        const std::size_t flen = ((static_cast<std::size_t>(raw[p+3] & 0x03)) << 11) |
                                 (static_cast<std::size_t>(raw[p+4]) << 3) |
                                 ((raw[p+5] >> 5) & 0x07);
        if (flen < 7 || p + flen > raw.size()) break;
        frames.emplace_back(raw.begin() + static_cast<std::ptrdiff_t>(p + 7),
                            raw.begin() + static_cast<std::ptrdiff_t>(p + flen));
        p += flen;
    }
    return frames;
}
} // namespace

int main()
{
    std::printf("=== 音视频 PS 往返 ===\n");

    const auto vraw = read_file("/tmp/test_h264_aud.h264");
    const auto araw = read_file("/tmp/test.aac");
    if (vraw.empty() || araw.empty()) { std::printf("跳过\n"); return 0; }

    const auto aframes = strip_adts(araw);
    std::error_code ec;

    // ---- 切视频帧 ----
    EsSplitter::Config sc; sc.codec = Codec::h264;
    auto sp = EsSplitter::create(sc, ec);
    std::vector<std::vector<std::uint8_t>> vframes;
    auto push_v = [&](span<const std::uint8_t> fr) {
        vframes.emplace_back(fr.begin(), fr.end());
    };
    sp->feed(span<const std::uint8_t>(vraw.data(), vraw.size()), push_v);
    sp->flush(push_v);

    // ---- 封 PS ----
    std::vector<std::uint8_t> ps;
    MuxerConfig mc; mc.container = Container::ps;
    auto mx = Muxer::create(mc, [&ps](span<const std::uint8_t> d) {
        ps.insert(ps.end(), d.begin(), d.end());
    }, ec);

    StreamConfig vs; vs.codec = Codec::h264;
    auto vsid = mx->addStream(vs, ec);

    StreamConfig as; as.codec = Codec::aac; as.add_adts = true;
    const std::uint8_t asc[] = {0x12, 0x10};
    as.extradata = span<const std::uint8_t>(asc, sizeof(asc));
    auto asid = mx->addStream(as, ec);

    std::size_t vi = 0, ai = 0;
    std::int64_t vp = 0, ap = 0;
    while (vi < vframes.size() || ai < aframes.size()) {
        if (vi < vframes.size() && (ai >= aframes.size() || vp <= ap)) {
            mx->write(*vsid, vi == 0 ? FrameFlag::key : FrameFlag::none, vp, vp,
                      span<const std::uint8_t>(vframes[vi].data(), vframes[vi].size()));
            vp += 3600; ++vi;
        } else {
            mx->write(*asid, FrameFlag::key, ap, ap,
                      span<const std::uint8_t>(aframes[ai].data(), aframes[ai].size()));
            ap += 2089; ++ai;
        }
    }
    mx->flush();
    std::printf("  封包: 视频 %zu 帧 + 音频 %zu 帧 → %zu 字节\n",
                vframes.size(), aframes.size(), ps.size());

    // ---- 解回 ----
    struct Out { std::vector<std::uint8_t> data; Codec codec; };
    std::vector<Out> outs;
    std::size_t stream_cbs = 0;

    DemuxerConfig dc; dc.container = Container::ps;
    auto dx = Demuxer::create(dc,
        [&](const FrameInfo& fi, span<const std::uint8_t> es) {
            outs.push_back({std::vector<std::uint8_t>(es.begin(), es.end()), fi.codec});
        },
        [&](const StreamInfo&) { ++stream_cbs; },
        ec);

    const std::size_t chunk = 512;
    for (std::size_t off = 0; off < ps.size(); off += chunk) {
        const std::size_t n = std::min(chunk, ps.size() - off);
        dx->feed(span<const std::uint8_t>(ps.data() + off, n));
    }
    dx->flush();

    // ---- 统计 ----
    std::vector<Out> outs_v, outs_a;
    for (auto& o : outs) {
        (isVideo(o.codec) ? outs_v : outs_a).push_back(std::move(o));
    }
    std::printf("  解出: 视频 %zu 帧，音频 %zu 帧，stream 回调 %zu 次\n",
                outs_v.size(), outs_a.size(), stream_cbs);

    EXPECT(outs_v.size() == vframes.size(), "视频帧数一致");
    EXPECT(outs_a.size() == aframes.size(), "音频帧数一致");
    EXPECT(stream_cbs == 2, "两个流各触发一次 stream 回调");

    // 视频逐字节
    std::size_t vmis = 0;
    for (std::size_t i = 0; i < std::min(outs_v.size(), vframes.size()); ++i) {
        if (outs_v[i].data != vframes[i]) ++vmis;
    }
    EXPECT(vmis == 0, "视频逐字节一致");

    // 音频：payload 应带 ADTS（7 字节头 + 原始裸帧）
    std::size_t amis = 0;
    for (std::size_t i = 0; i < std::min(outs_a.size(), aframes.size()); ++i) {
        const auto& o = outs_a[i].data;
        const auto& src = aframes[i];
        if (o.size() != src.size() + 7) { ++amis; continue; }
        if (o[0] != 0xFF || (o[1] & 0xF0) != 0xF0) { ++amis; continue; }   // ADTS sync
        if (std::memcmp(o.data() + 7, src.data(), src.size()) != 0) ++amis;
    }
    EXPECT(amis == 0, "音频 payload = 7 字节 ADTS + 原始裸帧");

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
