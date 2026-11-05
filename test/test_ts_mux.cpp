// =============================================================================
//  test_ts_mux.cpp —— H264 → TS
// =============================================================================
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <vector>
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
    std::printf("=== H264 → TS ===\n");

    const auto raw = read_file("/tmp/test_h264_aud.h264");
    if (raw.empty()) { std::printf("跳过\n"); return 0; }

    std::error_code ec;

    EsSplitter::Config sc; sc.codec = Codec::h264;
    auto sp = EsSplitter::create(sc, ec);

    std::vector<std::uint8_t> ts;
    MuxerConfig mc;
    mc.container = Container::ts;
    auto mx = Muxer::create(mc, [&ts](span<const std::uint8_t> d) {
        ts.insert(ts.end(), d.begin(), d.end());
    }, ec);
    EXPECT(mx != nullptr, "复用器");

    StreamConfig st; st.codec = Codec::h264;
    auto sid = mx->addStream(st, ec);
    EXPECT(sid.has_value(), "addStream");

    std::int64_t pts = 0;
    std::size_t frames = 0;
    auto on_frame = [&](span<const std::uint8_t> fr) {
        mx->write(*sid, frames == 0 ? FrameFlag::key : FrameFlag::none,
                  pts, pts, fr);
        pts += 3600;
        ++frames;
    };
    sp->feed(span<const std::uint8_t>(raw.data(), raw.size()), on_frame);
    sp->flush(on_frame);
    mx->flush();

    std::printf("  %zu 帧 → TS %zu 字节（%zu 个包）\n",
                frames, ts.size(), ts.size() / 188);

    EXPECT(!ts.empty(), "TS 非空");
    EXPECT(ts.size() % 188 == 0, "总长是 188 的整数倍");

    // 每包首字节必须是 0x47
    std::size_t bad = 0;
    for (std::size_t off = 0; off < ts.size(); off += 188) {
        if (ts[off] != 0x47) ++bad;
    }
    EXPECT(bad == 0, "每包 sync_byte = 0x47");

    std::FILE* f = std::fopen("/tmp/out.ts", "wb");
    if (f) { std::fwrite(ts.data(), 1, ts.size(), f); std::fclose(f); }
    std::printf("  已写 /tmp/out.ts\n");

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
