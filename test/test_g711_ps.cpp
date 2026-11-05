// =============================================================================
//  test_g711_ps.cpp —— G.711A/U 在 PS 里怎么放、能不能解回来
// =============================================================================
//  G.711 不是 MPEG 音频，PS 里没有对应的 stream_id 段（0xC0..0xDF 是 MPEG
//  音频专用），规范做法是塞进 private_stream_1（0xBD），靠 PSM 的
//  stream_type 0x90/0x91 告诉对端"这是 A-law / μ-law"。
//
//  这个测试盯三件事：
//    1. 我们自己能不能往返（码流逐字节一致、codec 认得回来）
//    2. ffmpeg 能不能认（对外部工具装不认识 = 白封）
//    3. PSM 的 elementary_stream_map_length 写对了没有
//
//  第 3 点是这个用例存在的真正理由：视频靠 0xE0 猜、AAC 靠 ADTS 同步字猜，
//  就算 PSM 整张表是坏的，ffprobe 也会给出"正确"的答案。只有 G.711 这种
//  只能靠 PSM 认的流能把它暴露出来 —— 早先按"条数"而不是"字节数"写这个
//  长度字段，两轮测试都没发现，就是这里炸的。
//
//  ⚠️ ffmpeg 侧已知：A-law 认成 pcm_alaw（8 kHz / 单声道，正确）；
//     μ-law 被认成 mp2 —— ffmpeg 的 PS 解复用没做 0x91 映射，与码流无关
//     （本库两种都能认回来）。对端是 ffmpeg 时建议用 A-law。
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <utility>

using namespace mpegmech;

namespace {

int g_fail = 0;
#define EXPECT(c, m) do { if (!(c)) { std::printf("  [FAIL] %s (line %d)\n", (m), __LINE__); ++g_fail; } } while (0)

/// 造一段 G.711 裸流：8 kHz × 20 ms = 160 字节/帧，锯齿波（能看出错位）。
std::vector<std::uint8_t> make_g711(std::size_t frames)
{
    std::vector<std::uint8_t> out;
    out.reserve(frames * 160);
    std::uint8_t v = 0;
    for (std::size_t n = 0; n < frames; ++n) {
        for (int i = 0; i < 160; ++i) {
            out.push_back(static_cast<std::uint8_t>(v + i));
        }
        v = static_cast<std::uint8_t>(v + 7);
    }
    return out;
}

struct OutFrame {
    Codec                     codec = Codec::unknown;
    std::int64_t              pts   = 0;
    std::vector<std::uint8_t> data;
};

std::string g_data_dir =
#if defined(_WIN32)
    ".";
#else
    "/tmp";      // 与其它测试保持一致的落盘位置，别往工作目录扔垃圾
#endif

} // namespace

int main()
{
    std::printf("=== G.711 进 PS ===\n");

    const std::size_t kFrames = 50;                 // 1 秒
    const auto        es      = make_g711(kFrames);

    for (Codec codec : {Codec::g711a, Codec::g711u}) {
        const char* name = (codec == Codec::g711a) ? "g711a" : "g711u";
        std::printf("-- %s --\n", name);

        std::vector<std::uint8_t> ps;
        std::error_code           ec;

        MuxerConfig mc;
        mc.container = Container::ps;
        auto mx = Muxer::create(mc, [&ps](span<const std::uint8_t> d) {
            ps.insert(ps.end(), d.begin(), d.end());
        }, ec);
        EXPECT(mx != nullptr, "复用器");

        StreamConfig sc;
        sc.codec = codec;
        auto sid = mx->addStream(sc, ec);
        EXPECT(sid.has_value(), "addStream");

        std::vector<std::vector<std::uint8_t>> src;
        std::int64_t pts = 0;
        for (std::size_t n = 0; n < kFrames; ++n) {
            span<const std::uint8_t> f(es.data() + n * 160, 160);
            src.emplace_back(f.begin(), f.end());
            auto w = mx->write(*sid, FrameFlag::key, pts, pts, f);
            EXPECT(!w, "write");
            pts += 20 * 90;                          // 20 ms = 1800 @90 kHz
        }
        mx->flush();

        std::printf("  %zu 帧 → PS %zu 字节\n", kFrames, ps.size());

        // 落盘给 ffmpeg 验
        if (const char* d = std::getenv("MPEGMECH_TEST_DATA")) { g_data_dir = d; }
        const std::string path = g_data_dir + "/out_" + name + ".ps";
        if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
            std::fwrite(ps.data(), 1, ps.size(), f);
            std::fclose(f);
            std::printf("  已写 %s\n", path.c_str());
        }

        // ---- 解回来 ----
        std::vector<OutFrame> got;
        DemuxerConfig         dc;
        dc.container = Container::unknown;
        auto dx = Demuxer::create(dc,
            [&got](const FrameInfo& fi, span<const std::uint8_t> payload) {
                OutFrame o;
                o.codec = fi.codec;
                o.pts   = fi.pts;
                o.data.assign(payload.begin(), payload.end());
                got.push_back(std::move(o));
            }, nullptr, ec);
        EXPECT(dx != nullptr, "解封装器");

        std::size_t i = 0;
        while (i < ps.size()) {                       // 块长不对齐
            std::size_t n = 37;
            if (n > ps.size() - i) { n = ps.size() - i; }
            dx->feed(span<const std::uint8_t>(ps.data() + i, n));
            i += n;
        }
        dx->flush();

        std::printf("  解出 %zu 帧\n", got.size());
        EXPECT(got.size() == src.size(), "帧数一致");
        if (got.size() == src.size()) {
            std::size_t bad_codec = 0, bad_data = 0, bad_pts = 0;
            for (std::size_t k = 0; k < got.size(); ++k) {
                if (got[k].codec != codec) { ++bad_codec; }
                if (got[k].data != src[k]) { ++bad_data; }
                if (k > 0 && got[k].pts - got[k - 1].pts != 1800) { ++bad_pts; }
            }
            EXPECT(bad_codec == 0, "codec 认回来");
            EXPECT(bad_data == 0, "码流逐字节一致");
            EXPECT(bad_pts == 0, "PTS 间隔恒为 20 ms");
        }
        EXPECT(dx->container() == Container::ps, "嗅探出 PS");
    }

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
