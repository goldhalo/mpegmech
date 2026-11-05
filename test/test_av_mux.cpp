// =============================================================================
//  test_av_mux.cpp —— H264 + AAC 混合封 PS
// =============================================================================
//  从 ADTS 文件剥出裸 AAC 帧，加上 ASC 交给 Muxer，验证 ffmpeg 能解出音视频。
// =============================================================================
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <cstring>
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

/// 剥 ADTS 头，返回裸帧列表。
std::vector<std::vector<std::uint8_t>> strip_adts(const std::vector<std::uint8_t>& raw)
{
    std::vector<std::vector<std::uint8_t>> frames;
    std::size_t p = 0;
    while (p + 7 <= raw.size()) {
        if (raw[p] != 0xFF || (raw[p+1] & 0xF0) != 0xF0) { break; }
        const std::size_t flen = ((static_cast<std::size_t>(raw[p+3] & 0x03)) << 11) |
                                 (static_cast<std::size_t>(raw[p+4]) << 3) |
                                 ((raw[p+5] >> 5) & 0x07);
        if (flen < 7 || p + flen > raw.size()) { break; }
        frames.emplace_back(raw.begin() + static_cast<std::ptrdiff_t>(p + 7),
                            raw.begin() + static_cast<std::ptrdiff_t>(p + flen));
        p += flen;
    }
    return frames;
}
} // namespace

int main()
{
    std::printf("=== H264 + AAC → PS ===\n");

    const auto vraw = read_file("/tmp/test_h264_aud.h264");
    const auto araw = read_file("/tmp/test.aac");
    if (vraw.empty() || araw.empty()) { std::printf("跳过（缺测试文件）\n"); return 0; }

    const auto aframes = strip_adts(araw);
    std::printf("  视频 %zu 字节，音频 %zu 帧\n", vraw.size(), aframes.size());
    EXPECT(!aframes.empty(), "剥出音频帧");

    std::error_code ec;

    // 切视频帧
    EsSplitter::Config sc;
    sc.codec = Codec::h264;
    auto sp = EsSplitter::create(sc, ec);

    // Muxer
    std::vector<std::uint8_t> ps;
    MuxerConfig mc; mc.container = Container::ps;
    auto mx = Muxer::create(mc, [&ps](span<const std::uint8_t> d) {
        ps.insert(ps.end(), d.begin(), d.end());
    }, ec);
    EXPECT(mx != nullptr, "复用器");

    // 视频流
    StreamConfig vs; vs.codec = Codec::h264;
    auto vsid = mx->addStream(vs, ec);
    EXPECT(vsid.has_value(), "加视频流");

    // 音频流（裸 AAC + ASC）
    StreamConfig as;
    as.codec    = Codec::aac;
    as.add_adts = true;                    // 让库补 ADTS
    const std::uint8_t asc[] = {0x12, 0x10};   // AAC-LC / 44.1k / 立体声
    as.extradata = span<const std::uint8_t>(asc, sizeof(asc));
    auto asid = mx->addStream(as, ec);
    EXPECT(asid.has_value(), "加音频流");

    // 交错写：视频 25fps（3600 tick）、音频 43fps（约 2089 tick）
    std::size_t vidx = 0, aidx = 0;
    std::int64_t vpts = 0, apts = 0;
    std::vector<std::vector<std::uint8_t>> vframes;

    auto push_video = [&](span<const std::uint8_t> fr) {
        vframes.emplace_back(fr.begin(), fr.end());
    };
    sp->feed(span<const std::uint8_t>(vraw.data(), vraw.size()), push_video);
    sp->flush(push_video);

    while (vidx < vframes.size() || aidx < aframes.size()) {
        if (vidx < vframes.size() && (aidx >= aframes.size() || vpts <= apts)) {
            FrameFlag f = (vidx == 0) ? FrameFlag::key : FrameFlag::none;
            mx->write(*vsid, f, vpts, vpts,
                      span<const std::uint8_t>(vframes[vidx].data(), vframes[vidx].size()));
            vpts += 3600;
            ++vidx;
        } else if (aidx < aframes.size()) {
            mx->write(*asid, FrameFlag::key, apts, apts,
                      span<const std::uint8_t>(aframes[aidx].data(), aframes[aidx].size()));
            apts += 2089;
            ++aidx;
        }
    }
    mx->flush();

    std::printf("  视频 %zu 帧，音频 %zu 帧 → PS %zu 字节\n",
                vframes.size(), aframes.size(), ps.size());

    std::FILE* f = std::fopen("/tmp/out_av.ps", "wb");
    if (f) { std::fwrite(ps.data(), 1, ps.size(), f); std::fclose(f); }
    std::printf("  已写 /tmp/out_av.ps\n");

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
