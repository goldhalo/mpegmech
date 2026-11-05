// =============================================================================
//  test_ps_mux.cpp —— H264 → PS 端到端
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
    if (!f) { return out; }
    std::uint8_t b[65536]; std::size_t n;
    while ((n = std::fread(b, 1, sizeof(b), f)) > 0) out.insert(out.end(), b, b + n);
    std::fclose(f);
    return out;
}
} // namespace

int main()
{
    std::printf("=== PS 复用端到端 ===\n");

    const auto raw = read_file("/tmp/test_h264_aud.h264");
    if (raw.empty()) { std::printf("跳过：没有测试文件\n"); return 0; }

    // 1. 切帧
    EsSplitter::Config sc;
    sc.codec = Codec::h264;
    std::error_code ec;
    auto sp = EsSplitter::create(sc, ec);
    EXPECT(sp != nullptr, "切帧器创建");

    // 2. 复用
    MuxerConfig mc;
    mc.container = Container::ps;

    std::vector<std::uint8_t> ps;
    auto mx = Muxer::create(mc, [&ps](span<const std::uint8_t> d) {
        ps.insert(ps.end(), d.begin(), d.end());
    }, ec);
    EXPECT(mx != nullptr, "复用器创建");

    StreamConfig st;
    st.codec = Codec::h264;
    auto sid = mx->addStream(st, ec);
    EXPECT(sid.has_value(), "addStream 成功");
    if (!sid) { return 1; }

    // 3. 串起来
    std::int64_t pts = 0;
    std::size_t frames = 0;
    auto on_frame = [&](span<const std::uint8_t> fr) {
        // 第一帧当关键帧（含 IDR）
        FrameFlag f = (frames == 0) ? FrameFlag::key : FrameFlag::none;
        mx->write(*sid, f, pts, pts, fr);
        pts += 3600;                      // 25fps
        ++frames;
    };

    sp->feed(span<const std::uint8_t>(raw.data(), raw.size()), on_frame);
    sp->flush(on_frame);
    mx->flush();

    std::printf("  帧数 %zu，PS 输出 %zu 字节\n", frames, ps.size());
    EXPECT(frames == 75, "应切出 75 帧");
    EXPECT(!ps.empty(), "PS 非空");

    // 4. 校验 PS 起始码存在
    bool has_pack = false, has_psm = false, has_pes = false;
    for (std::size_t i = 0; i + 4 < ps.size(); ++i) {
        if (ps[i] == 0 && ps[i+1] == 0 && ps[i+2] == 1) {
            if (ps[i+3] == 0xBA) has_pack = true;
            if (ps[i+3] == 0xBC) has_psm = true;
            if (ps[i+3] == 0xE0) has_pes = true;
        }
    }
    EXPECT(has_pack, "有 PackHeader (0xBA)");
    EXPECT(has_psm,  "有 PSM (0xBC)");
    EXPECT(has_pes,  "有 PES (0xE0)");

    // 5. 写文件给 ffmpeg 验
    std::FILE* f = std::fopen("/tmp/out.ps", "wb");
    if (f) { std::fwrite(ps.data(), 1, ps.size(), f); std::fclose(f); }
    std::printf("  已写 /tmp/out.ps（%zu 字节）\n", ps.size());

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
