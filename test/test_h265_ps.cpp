// =============================================================================
//  test_h265_ps.cpp —— H265 → PS 往返
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <system_error>

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
    std::printf("=== H265 → PS 往返 ===\n");
    const auto raw = read_file("/tmp/test_nob.h265");
    if (raw.empty()) { std::printf("跳过\n"); return 0; }

    std::error_code ec;

    EsSplitter::Config sc;
    sc.codec = Codec::h265;
    auto sp = EsSplitter::create(sc, ec);
    EXPECT(sp != nullptr, "切帧器");

    std::vector<std::uint8_t> ps;
    MuxerConfig mc; mc.container = Container::ps;
    auto mx = Muxer::create(mc, [&ps](span<const std::uint8_t> d) {
        ps.insert(ps.end(), d.begin(), d.end());
    }, ec);
    EXPECT(mx != nullptr, "复用器");

    StreamConfig st; st.codec = Codec::h265;
    auto sid = mx->addStream(st, ec);
    EXPECT(sid.has_value(), "addStream");

    std::vector<std::vector<std::uint8_t>> orig;
    std::int64_t pts = 0;
    auto on_frame = [&](span<const std::uint8_t> fr) {
        orig.emplace_back(fr.begin(), fr.end());
        FrameFlag f = (orig.size() == 1) ? FrameFlag::key : FrameFlag::none;
        mx->write(*sid, f, pts, pts, fr);
        pts += 3600;
    };
    sp->feed(span<const std::uint8_t>(raw.data(), raw.size()), on_frame);
    sp->flush(on_frame);
    mx->flush();

    std::printf("  原始 %zu 帧 → PS %zu 字节\n", orig.size(), ps.size());

    std::FILE* f = std::fopen("/tmp/out_h265.ps", "wb");
    if (f) { std::fwrite(ps.data(), 1, ps.size(), f); std::fclose(f); }

    // 解回
    std::vector<std::vector<std::uint8_t>> out;
    DemuxerConfig dc; dc.container = Container::ps;
    auto dx = Demuxer::create(dc,
        [&](const FrameInfo&, span<const std::uint8_t> es) {
            out.emplace_back(es.begin(), es.end());
        }, nullptr, ec);
    EXPECT(dx != nullptr, "解封装器");

    const std::size_t chunk = 777;
    for (std::size_t off = 0; off < ps.size(); off += chunk) {
        const std::size_t n = std::min(chunk, ps.size() - off);
        dx->feed(span<const std::uint8_t>(ps.data() + off, n));
    }
    dx->flush();

    std::printf("  解出 %zu 帧\n", out.size());
    EXPECT(out.size() == orig.size(), "帧数一致");

    std::size_t mismatch = 0;
    for (std::size_t i = 0; i < std::min(out.size(), orig.size()); ++i) {
        if (out[i] != orig[i]) { ++mismatch; }
    }
    EXPECT(mismatch == 0, "逐字节一致");

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
