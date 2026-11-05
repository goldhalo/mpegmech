// =============================================================================
//  test_ps_oversize.cpp —— PS 的单帧上限（PES_packet_length 只有 16 位）
//
//  这条以前是零覆盖：喂一个 > 64 KiB 的帧进 PS 复用器，它会**静默**把
//  PES_packet_length 写成 0（"不定长"），产出一条 ffmpeg 读不了的流
//  （"could not find codec parameters"），而 write() 返回 0 ——
//  调用方完全不知情。
//
//  1080p 的 IDR 普遍 50~200 KB，超过 65535 是常态而不是例外，所以这条
//  在 GB28181 主场景里是**必然**会踩到的。
//
//  本用例不依赖任何外部素材。
// =============================================================================
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

/// 在 PS 输出里找第一条 PES（00 00 01 E0）的 PES_packet_length。
/// @return -1 = 没找到
int pes_length_of(const std::vector<std::uint8_t>& ps)
{
    for (std::size_t i = 0; i + 6 < ps.size(); ++i) {
        if (ps[i] == 0x00 && ps[i + 1] == 0x00 && ps[i + 2] == 0x01 &&
            ps[i + 3] == 0xE0) {
            return (static_cast<int>(ps[i + 4]) << 8) | ps[i + 5];
        }
    }
    return -1;
}

/// 一次写入：返回 error_code，并记录回调是否被调过。
struct Run {
    std::error_code              ec;
    bool                         called = false;
    std::vector<std::uint8_t>    out;
    MuxStats                   stats{};
    int                          sid = -1;
};

Run write_one(std::size_t payload_len, std::int64_t pts = 0)
{
    Run r;

    MuxerConfig mc;
    mc.container = Container::ps;

    auto mx = Muxer::create(mc, [&r](span<const std::uint8_t> d) {
        r.called = true;
        r.out.insert(r.out.end(), d.begin(), d.end());
    }, r.ec);
    if (!mx || r.ec) {
        return r;
    }

    StreamConfig sc;
    sc.codec = Codec::h264;
    const auto sid = mx->addStream(sc, r.ec);
    if (!sid || r.ec) {
        return r;
    }
    r.sid = *sid;

    std::vector<std::uint8_t> es(payload_len, 0x5A);
    // H.264 IDR 起始码，顺带让"关键帧 → 重发 PS 头"的路径也走一遍
    if (payload_len >= 5) {
        es[0] = 0x00; es[1] = 0x00; es[2] = 0x00; es[3] = 0x01; es[4] = 0x65;
    }

    r.ec = mx->write(r.sid, FrameFlag::key, pts, pts,
                     span<const std::uint8_t>(es.data(), es.size()));
    r.stats = mx->stats();
    return r;
}

} // namespace

int main()
{
    std::printf("=== PS 单帧上限 ===\n");

    // 头部 8 字节（3 flags + 5 PTS），所以 payload 上限是 65535 - 8。
    constexpr std::size_t kMaxPayload = 0xFFFF - 8;

    // ---- 1. 正常帧：长度字段必须是真实值 ----
    {
        const auto r = write_one(1000);
        EXPECT(!r.ec, "普通大小的帧应当成功");
        EXPECT(r.called, "普通帧应当交出字节");
        const int len = pes_length_of(r.out);
        EXPECT(len == static_cast<int>(8 + 1000),
               "PES_packet_length 应当是真实的 8+payload");
        EXPECT(r.stats.oversize_drops == 0, "普通帧不该计入 oversize_drops");
    }

    // ---- 2. 刚好装得下的边界 ----
    {
        const auto r = write_one(kMaxPayload);
        EXPECT(!r.ec, "恰好 65535 的帧应当成功");
        const int len = pes_length_of(r.out);
        EXPECT(len == 0xFFFF, "边界的长度字段应当是 65535（不是 0）");
    }

    // ---- 3. 超一个字节就必须报错 ----
    {
        const auto r = write_one(kMaxPayload + 1);
        EXPECT(r.ec == std::make_error_code(std::errc::invalid_argument) ||
               r.ec == make_error_code(MpegErrc::invalid_argument),
               "超限帧应当返回 invalid_argument");
        EXPECT(!r.called, "超限帧不该交出任何字节（宁可没有，也不要非法 PS）");
        EXPECT(r.stats.oversize_drops == 1, "应当计入 oversize_drops");
    }

    // ---- 4. 典型 1080p IDR（200 KB）----
    {
        const auto r = write_one(200 * 1024);
        EXPECT(r.ec == make_error_code(MpegErrc::invalid_argument),
               "200 KB 的 IDR 应当被拒绝（PS 装不下）");
        EXPECT(r.stats.oversize_drops == 1, "200 KB 帧应当计入 oversize_drops");
        EXPECT(r.out.empty(), "不该产出任何字节");
    }

    // ---- 5. TS 不受影响：那边用不定长 PES 是合法的 ----
    {
        std::error_code ec;
        std::size_t     total = 0;

        MuxerConfig mc;
        mc.container = Container::ts;

        auto mx = Muxer::create(mc, [&total](span<const std::uint8_t> d) {
            total += d.size();
        }, ec);
        EXPECT(mx != nullptr && !ec, "TS 复用器创建");

        if (mx) {
            StreamConfig sc;
            sc.codec = Codec::h264;
            const auto sid = mx->addStream(sc, ec);
            EXPECT(sid.has_value(), "TS 加流");

            std::vector<std::uint8_t> es(200 * 1024, 0x5A);
            const auto e = mx->write(sid.value_or(0), FrameFlag::key, 0, 0,
                                     span<const std::uint8_t>(es.data(), es.size()));
            EXPECT(!e, "TS 下 200 KB 的帧应当正常写出（PES 可以是不定长）");
            EXPECT(total > 200 * 1024, "TS 输出字节数应当不小于载荷");
            EXPECT(mx->stats().oversize_drops == 0, "TS 不该计入 oversize_drops");
        }
    }

    if (g_fail == 0) {
        std::printf("PASS（0 个失败）\n");
        return 0;
    }
    std::printf("FAIL（%d 个失败）\n", g_fail);
    return 1;
}
