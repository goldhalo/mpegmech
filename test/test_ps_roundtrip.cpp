// =============================================================================
//  test_ps_roundtrip.cpp —— H264 → PS → H264 往返一致性
// =============================================================================
//  这是最严格的验证：如果 mux 和 demux 两侧都对，解出来的 ES 应该跟原始
//  裸流**逐字节一致**（PS 只是给 ES 加了一层壳，不改内容）。
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <cstring>
#include <vector>

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
} // namespace

int main()
{
    std::printf("=== PS 往返一致性 ===\n");

    const auto raw = read_file("/tmp/test_h264_aud.h264");
    if (raw.empty()) { std::printf("跳过\n"); return 0; }

    std::error_code ec;

    // ---- 1. 切帧 ----
    EsSplitter::Config sc;
    sc.codec = Codec::h264;
    auto sp = EsSplitter::create(sc, ec);
    EXPECT(sp != nullptr, "切帧器");

    // ---- 2. 封 PS ----
    std::vector<std::uint8_t> ps;
    MuxerConfig mc; mc.container = Container::ps;
    auto mx = Muxer::create(mc, [&ps](span<const std::uint8_t> d) {
        ps.insert(ps.end(), d.begin(), d.end());
    }, ec);
    EXPECT(mx != nullptr, "复用器");

    StreamConfig st; st.codec = Codec::h264;
    auto sid = mx->addStream(st, ec);
    EXPECT(sid.has_value(), "addStream");

    // 记录原始每帧（用于比对）
    std::vector<std::vector<std::uint8_t>> orig_frames;
    std::int64_t pts = 0;
    auto on_frame = [&](span<const std::uint8_t> fr) {
        orig_frames.emplace_back(fr.begin(), fr.end());
        FrameFlag f = (orig_frames.size() == 1) ? FrameFlag::key : FrameFlag::none;
        mx->write(*sid, f, pts, pts, fr);
        pts += 3600;
    };
    sp->feed(span<const std::uint8_t>(raw.data(), raw.size()), on_frame);
    sp->flush(on_frame);
    mx->flush();

    std::printf("  原始 %zu 帧 → PS %zu 字节\n", orig_frames.size(), ps.size());

    // ---- 3. 解 PS ----
    std::vector<std::vector<std::uint8_t>> out_frames;
    std::vector<std::int64_t> out_pts;
    DemuxerConfig dc;
    dc.container = Container::ps;
    auto dx = Demuxer::create(dc,
        [&](const FrameInfo& fi, span<const std::uint8_t> es) {
            out_frames.emplace_back(es.begin(), es.end());
            out_pts.push_back(fi.pts);
        },
        nullptr, ec);
    EXPECT(dx != nullptr, "解封装器");

    // 分块喂（模拟网络）
    const std::size_t chunk = 1000;
    for (std::size_t off = 0; off < ps.size(); off += chunk) {
        const std::size_t n = std::min(chunk, ps.size() - off);
        dx->feed(span<const std::uint8_t>(ps.data() + off, n));
    }
    dx->flush();

    std::printf("  解出 %zu 帧\n", out_frames.size());
    EXPECT(out_frames.size() == orig_frames.size(), "帧数一致");

    // ---- 4. 逐帧逐字节比对 ----
    std::size_t mismatch = 0;
    const std::size_t n = std::min(out_frames.size(), orig_frames.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (out_frames[i] != orig_frames[i]) {
            ++mismatch;
            if (mismatch <= 2) {
                std::printf("  帧 %zu 不一致: 原 %zu 字节, 解 %zu 字节\n",
                            i, orig_frames[i].size(), out_frames[i].size());
            }
        }
    }
    EXPECT(mismatch == 0, "每帧逐字节一致");

    // ---- 5. PTS 也应一致 ----
    bool pts_ok = true;
    for (std::size_t i = 0; i < n; ++i) {
        if (out_pts[i] != static_cast<std::int64_t>(i * 3600)) { pts_ok = false; break; }
    }
    EXPECT(pts_ok, "PTS 连续（0, 3600, 7200...）");

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
