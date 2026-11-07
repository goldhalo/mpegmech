// =============================================================================
//  test_es_splitter.cpp
// =============================================================================
#include <mpegmech/es_splitter.hpp>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <system_error>


using namespace mpegmech;

namespace {

/// 极简的测试断言（mpegmech 还没有测试框架，先内联一个）。
#define EXPECT(cond, msg)                                                      \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  [FAIL] %s (line %d)\n", (msg), __LINE__);           \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

int g_fail = 0;

/// 把文件读成字节。
std::vector<std::uint8_t> read_file(const char* path)
{
    std::vector<std::uint8_t> out;
    std::FILE* f = std::fopen(path, "rb");
    if (!f) { return out; }
    std::uint8_t buf[64 * 1024];
    std::size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
        out.insert(out.end(), buf, buf + n);
    }
    std::fclose(f);
    return out;
}

/// 统计一个 AnnexB 缓冲里的帧数（用同样的切帧器）。
std::size_t count_frames(const std::vector<std::uint8_t>& raw, Codec codec,
                         std::size_t chunk /* 0 = 一次喂完 */)
{
    EsSplitter::Config cfg;
    cfg.codec = codec;
    std::error_code ec;
    auto sp = EsSplitter::create(cfg, ec);
    if (!sp) { return 0; }

    std::size_t frames = 0;
    auto cb = [&frames](span<const std::uint8_t>) { ++frames; };

    if (chunk == 0) {
        sp->feed(span<const std::uint8_t>(raw.data(), raw.size()), cb);
    } else {
        for (std::size_t off = 0; off < raw.size(); off += chunk) {
            const std::size_t n = std::min(chunk, raw.size() - off);
            sp->feed(span<const std::uint8_t>(raw.data() + off, n), cb);
        }
    }
    sp->flush(cb);
    return frames;
}

} // namespace

// -----------------------------------------------------------------------------
// 1. 带 AUD 的 H264：切出来的帧数应该等于 AUD 的个数
// -----------------------------------------------------------------------------
void test_h264_aud_frame_count()
{
    const auto raw = read_file("/tmp/test_h264_aud.h264");
    if (raw.empty()) {
        std::printf("  (跳过：没有 /tmp/test_h264_aud.h264)\n");
        return;
    }

    // 数 AUD（NAL type 9）
    std::size_t aud_count = 0;
    for (std::size_t i = 0; i + 4 < raw.size(); ++i) {
        if (raw[i] == 0 && raw[i+1] == 0 && raw[i+2] == 1) {
            const std::uint8_t t = raw[i+3] & 0x1F;
            if (t == 9) { ++aud_count; }
        }
    }

    const std::size_t frames = count_frames(raw, Codec::h264, 0);
    std::printf("  H264(带AUD): AUD %zu 个，切出 %zu 帧\n", aud_count, frames);
    EXPECT(frames == aud_count, "帧数应等于 AUD 个数");
}

// -----------------------------------------------------------------------------
// 2. 分块喂 → 帧数必须一致（验证跨块起始码）
// -----------------------------------------------------------------------------
void test_chunked_equals_whole()
{
    const auto raw = read_file("/tmp/test_h264_aud.h264");
    if (raw.empty()) { return; }

    const std::size_t whole = count_frames(raw, Codec::h264, 0);
    const std::size_t c1    = count_frames(raw, Codec::h264, 1);
    const std::size_t c7    = count_frames(raw, Codec::h264, 7);
    const std::size_t c100  = count_frames(raw, Codec::h264, 100);

    std::printf("  一次喂完 %zu；按 1 字节 %zu；按 7 字节 %zu；按 100 字节 %zu\n",
                whole, c1, c7, c100);
    EXPECT(c1 == whole,   "1 字节切分应与整体一致");
    EXPECT(c7 == whole,   "7 字节切分应与整体一致");
    EXPECT(c100 == whole, "100 字节切分应与整体一致");
}

// -----------------------------------------------------------------------------
// 3. 非开头输入：前面的残片应该被丢弃并计数
// -----------------------------------------------------------------------------
void test_leading_partial_dropped()
{
    const auto raw = read_file("/tmp/test_h264_aud.h264");
    if (raw.empty()) { return; }

    EsSplitter::Config cfg;
    cfg.codec = Codec::h264;
    std::error_code ec;
    auto sp = EsSplitter::create(cfg, ec);

    std::size_t frames = 0;
    auto cb = [&frames](span<const std::uint8_t>) { ++frames; };

    // 从第 100 字节开始喂（大概率在帧中间）
    sp->feed(span<const std::uint8_t>(raw.data() + 100, raw.size() - 100), cb);
    sp->flush(cb);

    std::printf("  从 offset=100 开始：%zu 帧，丢弃残片 %zu 字节\n",
                frames, static_cast<std::size_t>(sp->stats().dropped_leading_bytes));
    EXPECT(frames > 0, "应该切出帧");
    EXPECT(sp->stats().dropped_leading_bytes > 0, "应该记录丢弃的残片");

    // 注意：这里**不能**断言"帧数 = AUD 数"。
    //
    // offset=100 之后第一个能当帧开头的 NAL 是一个 IDR（没有 AUD 标记），
    // 于是第一帧是"无 AUD 的 IDR 帧"，后面 74 个 AUD 各是一帧 —— 共 75。
    //
    // 这不算 bug：IDR 本身就是完整一帧，只是缺了 AUD 这个"分隔符"。真实
    // 设备送的裸流也经常这样（第一个 IDR 前面没有 AUD）。
    // 期望值从素材推导（整体切帧数 = 起始残片之后的完整帧数）：
    // 以前硬编码 75，换一份帧数不同的素材就必然假失败。
    const std::size_t expected = count_frames(raw, Codec::h264, 0);
    EXPECT(frames == expected, "起始残片之后的帧数应与整体一致");
}

// -----------------------------------------------------------------------------
// 4. H265
// -----------------------------------------------------------------------------
void test_h265()
{
    const auto raw = read_file("/tmp/test_nob.h265");
    if (raw.empty()) {
        std::printf("  (跳过：没有 /tmp/test_nob.h265)\n");
        return;
    }
    const std::size_t frames = count_frames(raw, Codec::h265, 0);
    std::printf("  H265: 切出 %zu 帧\n", frames);
    EXPECT(frames > 0, "H265 应该切出帧");
}

// -----------------------------------------------------------------------------
// 5. 参数校验
// -----------------------------------------------------------------------------
void test_invalid_config()
{
    std::error_code ec;
    EsSplitter::Config cfg;
    cfg.codec = Codec::h264;
    cfg.max_frame_bytes = 0;
    EXPECT(EsSplitter::create(cfg, ec) == nullptr, "max_frame_bytes=0 应被拒");

    cfg.max_frame_bytes = 1024;
    cfg.codec = Codec::aac;
    EXPECT(EsSplitter::create(cfg, ec) == nullptr, "非 H26x 应被拒");
}

int main()
{
    std::printf("=== EsSplitter 测试 ===\n");

    test_h264_aud_frame_count();
    test_chunked_equals_whole();
    test_leading_partial_dropped();
    test_h265();
    test_invalid_config();

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
