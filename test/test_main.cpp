// =============================================================================
//  test/test_main.cpp —— 单元测试（自带极简断言框架，无外部依赖）
// =============================================================================
//  文件    : test/test_main.cpp
//  用途    : 覆盖 M1 阶段已实现的部分：容器嗅探、时间基换算、帧标志。
//            解析器测试（TS/PS demux、mux、往返、切分不变性）在 M2~M6 补。
//  编码    : UTF-8
// =============================================================================

#include <mpegmech/mpegmech.hpp>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_pass = 0;
int g_fail = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
            ++g_fail;                                                           \
        } else {                                                                \
            ++g_pass;                                                           \
        }                                                                       \
    } while (0)

void section(const char* name)
{
    std::printf("[ %s ]\n", name);
}

using namespace mpegmech;

// -----------------------------------------------------------------------------
// 样本构造
// -----------------------------------------------------------------------------

/// 造一段"看起来像 TS"的数据：len 字节，在 base 处放 0x47，之后每 188 一个。
std::vector<std::uint8_t> makeTsSample(std::size_t lead, std::size_t packets)
{
    std::vector<std::uint8_t> buf(lead + packets * 188, 0x00);
    for (std::size_t i = 0; i < packets; ++i) {
        buf[lead + i * 188] = 0x47;
    }
    return buf;
}

/// 造一段 PS 数据：00 00 01 BA 开头，后面填任意内容。
std::vector<std::uint8_t> makePsSample(std::size_t lead)
{
    std::vector<std::uint8_t> buf(lead + 64, 0xAA);
    buf[lead + 0] = 0x00;
    buf[lead + 1] = 0x00;
    buf[lead + 2] = 0x01;
    buf[lead + 3] = 0xBA;
    return buf;
}

// -----------------------------------------------------------------------------
// 嗅探
// -----------------------------------------------------------------------------

void test_sniff()
{
    section("sniffContainer");

    // 1) 纯 TS，从 0 开始
    {
        auto d = makeTsSample(0, 3);
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::ts);
    }

    // 2) TS 前面带 37 字节垃圾（模拟中途接入 / 丢包后重入）
    {
        auto d = makeTsSample(37, 3);
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::ts);
    }

    // 3) TS 前缀里恰好有一个孤立的 0x47（对抗样本：不能因此误判）
    {
        auto d = makeTsSample(0, 3);
        d[10] = 0x47;   // 干扰字节
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::ts);
    }

    // 4) PS
    {
        auto d = makePsSample(0);
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::ps);
    }

    // 5) PS 前面带垃圾
    {
        auto d = makePsSample(123);
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::ps);
    }

    // 6) 垃圾数据
    {
        std::vector<std::uint8_t> d(600, 0x5A);
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::unknown);
    }

    // 7) 空输入
    {
        std::vector<std::uint8_t> d;
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::unknown);
    }

    // 8) 只有 2 个包（数据不足）—— 不能凭 2 个 0x47 就认定是 TS
    {
        auto d = makeTsSample(0, 2);
        CHECK(sniffContainer(span<const std::uint8_t>(d)) == Container::unknown);
    }
}

// -----------------------------------------------------------------------------
// 时间基
// -----------------------------------------------------------------------------

void test_time()
{
    section("time base");

    CHECK(msToClock(0) == 0);
    CHECK(msToClock(1) == 90);
    CHECK(msToClock(1000) == 90000);
    CHECK(msToClock(-5) == -450);

    CHECK(clockToMs(0) == 0);
    CHECK(clockToMs(90) == 1);
    CHECK(clockToMs(90000) == 1000);
    CHECK(clockToMs(89) == 0);     // 999.9 us -> 0 ms（向下取整）
    CHECK(clockToMs(91) == 1);
    CHECK(clockToMs(-90) == -1);
    CHECK(clockToMs(-91) == -2);   // 向下取整，不是向零

    // 往返一致性（整毫秒）
    for (std::int64_t ms = 0; ms < 5000; ms += 37) {
        CHECK(clockToMs(msToClock(ms)) == ms);
    }
}

// -----------------------------------------------------------------------------
// 帧标志
// -----------------------------------------------------------------------------

void test_flags()
{
    section("FrameFlag");

    const FrameFlag f = FrameFlag::key | FrameFlag::aud;
    CHECK(hasFlag(f, FrameFlag::key));
    CHECK(hasFlag(f, FrameFlag::aud));
    CHECK(!hasFlag(f, FrameFlag::config));
    CHECK(!hasFlag(FrameFlag::none, FrameFlag::key));

    FrameFlag g = FrameFlag::none;
    g |= FrameFlag::config;
    CHECK(hasFlag(g, FrameFlag::config));
    CHECK(!hasFlag(g, FrameFlag::key));
}

// -----------------------------------------------------------------------------
// 枚举 / 类型属性
// -----------------------------------------------------------------------------

void test_codec()
{
    section("Codec 属性");

    CHECK(isVideo(Codec::h264));
    CHECK(isVideo(Codec::h265));
    CHECK(!isVideo(Codec::aac));
    CHECK(isAudio(Codec::aac));
    CHECK(isAudio(Codec::g711a));
    CHECK(!isAudio(Codec::h264));

    CHECK(isAnnexB(Codec::h264));
    CHECK(isAnnexB(Codec::h265));
    CHECK(!isAnnexB(Codec::aac));

    CHECK(std::strcmp(toString(Container::ts), "ts") == 0);
    CHECK(std::strcmp(toString(Container::ps), "ps") == 0);
    CHECK(std::strcmp(toString(Codec::h264), "h264") == 0);
    CHECK(std::strcmp(toString(Codec::unknown), "unknown") == 0);
}

// -----------------------------------------------------------------------------
// span
// -----------------------------------------------------------------------------

void test_span()
{
    section("span");

    const std::uint8_t raw[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    span<const std::uint8_t> s(raw);

    CHECK(s.size() == 8);
    CHECK(s[3] == 3);
    CHECK(!s.empty());
    CHECK(s.first(3).size() == 3);
    CHECK(s.subspan(6, 10).size() == 2);   // 越界自动截断

    span<const std::uint8_t> empty;
    CHECK(empty.empty());
}

// -----------------------------------------------------------------------------
// 错误码
// -----------------------------------------------------------------------------

void test_error()
{
    section("error_code");

    const std::error_code ec = MpegErrc::unsupported_container;
    CHECK(ec == MpegErrc::unsupported_container);
    CHECK(ec.category() == mpeg_category());
    CHECK(!ec.message().empty());

    std::printf("  (样例) unsupported_container -> \"%s\"\n", ec.message().c_str());
}

} // namespace

int main()
{
    std::printf("=== mpegmech 单元测试（M1 阶段：工具层）===\n");

    test_sniff();
    test_time();
    test_flags();
    test_codec();
    test_span();
    test_error();

    std::printf("\n通过 %d，失败 %d\n", g_pass, g_fail);
    if (g_fail != 0) {
        std::printf("结果：FAIL\n");
        return 1;
    }
    std::printf("结果：OK\n");
    return 0;
}
