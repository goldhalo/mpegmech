// =============================================================================
//  test_p2_regressions.cpp —— 第二批 P2 修复的回归用例（全部自包含）
//
//  覆盖：
//    T1  PS/TS 的 PSI 版本号随 addStream 递增（PSM / PAT）
//    T3  span 空视图不做空指针算术（UBSan pointer-overflow）
//    T4  PSM 的 CRC32 不再写 0
//    T5  TS 路径 dts 归一、AAC 尾部计错误、PS flush 残帧按真实流标注
//    T6  addStream：容量上限 / 重复 PID、stream_id / 保留 PID
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/muxer.hpp>
#include <mpegmech/span.hpp>

#include "ps_muxer.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <system_error>
#include <vector>

using namespace mpegmech;

namespace {

int g_fail = 0;
#define EXPECT(c, m) do { if (!(c)) { std::printf("  [FAIL] %s (line %d)\n", (m), __LINE__); ++g_fail; } } while (0)

const std::uint8_t kFrame264[] = {0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x21};

/// 本地 MPEG-2 CRC32（与库内 crc32_mpeg2 相同的参数），用于独立核对。
std::uint32_t crc32_mpeg2_local(const std::uint8_t* data, std::size_t len)
{
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<std::uint32_t>(data[i]) << 24;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
    }
    return crc;
}

void put_timestamp(std::vector<std::uint8_t>& v, std::uint8_t prefix, std::int64_t ts)
{
    const std::uint64_t t = static_cast<std::uint64_t>(ts) & 0x1FFFFFFFFull;
    v.push_back(static_cast<std::uint8_t>((prefix << 4) | ((t >> 29) & 0x0E) | 0x01));
    v.push_back(static_cast<std::uint8_t>((t >> 22) & 0xFF));
    v.push_back(static_cast<std::uint8_t>(((t >> 14) & 0xFE) | 0x01));
    v.push_back(static_cast<std::uint8_t>((t >> 7) & 0xFF));
    v.push_back(static_cast<std::uint8_t>(((t << 1) & 0xFE) | 0x01));
}

/// 一次写入并收集输出。
struct MuxRun {
    std::error_code           ec;
    std::vector<std::uint8_t> out;
    std::vector<int>          ids;
};

MuxRun mux_write(Container c, const std::vector<StreamConfig>& streams,
                 int frames, std::int64_t pts, std::int64_t dts)
{
    MuxRun r;
    MuxerConfig mc;
    mc.container = c;
    auto mx = Muxer::create(mc, [&r](span<const std::uint8_t> d) {
        r.out.insert(r.out.end(), d.begin(), d.end());
    }, r.ec);
    if (!mx) { return r; }

    for (const auto& sc : streams) {
        auto id = mx->addStream(sc, r.ec);
        if (!id) { return r; }
        r.ids.push_back(*id);
    }
    for (int f = 0; f < frames; ++f) {
        const int sid = r.ids[static_cast<std::size_t>(f) % r.ids.size()];
        r.ec = mx->write(sid, f == 0 ? FrameFlag::key : FrameFlag::none,
                         pts + f * 3000, dts + f * 3000,
                         span<const std::uint8_t>(kFrame264, sizeof kFrame264));
        if (r.ec) { return r; }
    }
    mx->flush();
    return r;
}

int find_psm_version(const std::vector<std::uint8_t>& v)
{
    for (std::size_t i = 0; i + 7 <= v.size(); ++i) {
        if (v[i] == 0 && v[i + 1] == 0 && v[i + 2] == 1 && v[i + 3] == 0xBC) {
            return v[i + 6] & 0x1F;   // [BC][len hi][len lo][version byte]
        }
    }
    return -1;
}

int find_pat_version(const std::vector<std::uint8_t>& v)
{
    for (std::size_t i = 0; i + 188 <= v.size(); i += 188) {
        if (v[i] != 0x47) { continue; }
        const std::uint16_t pid = static_cast<std::uint16_t>(((v[i + 1] & 0x1F) << 8) | v[i + 2]);
        if (pid != 0x0000) { continue; }

        // 先跳过 adaptation field（PAT 只有十几字节，包场会用 AF 填充）
        std::size_t off = 4;
        const std::uint8_t afc = (v[i + 3] >> 4) & 0x03;
        if ((afc & 0x02) != 0) {
            off += 1 + v[i + 4];
        }
        if (off + 7 > 188) { continue; }
        const std::uint8_t* p = v.data() + i + off;
        // [0]pointer [1]table_id [2..3]len [4..5]tsid [6]version
        if (p[0] != 0x00 || p[1] != 0x00) { continue; }
        return (p[6] >> 1) & 0x1F;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// T1：PSI 版本号随 addStream 递增
// ---------------------------------------------------------------------------
void test_psi_version_increments()
{
    // ---- PS：PSM 版本 ----
    {
        MuxerConfig mc;
        mc.container = Container::ps;
        std::vector<std::uint8_t> out;
        std::error_code ec;
        auto mx = Muxer::create(mc, [&out](span<const std::uint8_t> d) {
            out.insert(out.end(), d.begin(), d.end());
        }, ec);
        if (!mx) { EXPECT(false, "PS 复用器创建"); return; }
        StreamConfig s1;
        s1.codec = Codec::h264;
        auto id1 = mx->addStream(s1, ec);
        EXPECT(id1.has_value(), "PS 第 1 条流");
        mx->write(*id1, FrameFlag::key, 0, 0, span<const std::uint8_t>(kFrame264, sizeof kFrame264));
        const int v1 = find_psm_version(out);
        EXPECT(v1 >= 0, "第一次输出应含 PSM");

        out.clear();
        StreamConfig s2;
        s2.codec = Codec::g711a;
        auto id2 = mx->addStream(s2, ec);
        EXPECT(id2.has_value(), "PS 第 2 条流");
        mx->write(*id2, FrameFlag::key, 9000, 9000,
                  span<const std::uint8_t>(kFrame264, sizeof kFrame264));
        const int v2 = find_psm_version(out);
        EXPECT(v2 >= 0, "第二次输出应含 PSM");
        EXPECT(v1 >= 0 && v2 == ((v1 + 1) & 0x1F), "PSM 版本号应 +1（曾恒为 0）");
    }

    // ---- TS：PAT 版本 ----
    {
        MuxerConfig mc;
        mc.container = Container::ts;
        std::vector<std::uint8_t> out;
        std::error_code ec;
        auto mx = Muxer::create(mc, [&out](span<const std::uint8_t> d) {
            out.insert(out.end(), d.begin(), d.end());
        }, ec);
        if (!mx) { EXPECT(false, "TS 复用器创建"); return; }
        StreamConfig s1;
        s1.codec = Codec::h264;
        auto id1 = mx->addStream(s1, ec);
        mx->write(*id1, FrameFlag::key, 0, 0, span<const std::uint8_t>(kFrame264, sizeof kFrame264));
        const int v1 = find_pat_version(out);
        EXPECT(v1 >= 0, "第一次输出应含 PAT");

        out.clear();
        StreamConfig s2;
        s2.codec = Codec::aac;
        auto id2 = mx->addStream(s2, ec);
        EXPECT(id2.has_value(), "TS 第 2 条流");
        mx->write(*id2, FrameFlag::key, 9000, 9000,
                  span<const std::uint8_t>(kFrame264, sizeof kFrame264));
        const int v2 = find_pat_version(out);
        EXPECT(v2 >= 0, "第二次输出应含 PAT");
        EXPECT(v1 >= 0 && v2 == ((v1 + 1) & 0x1F), "PAT 版本号应 +1（曾恒为 0）");
    }
}

// ---------------------------------------------------------------------------
// T3：空 span 的 end()/subspan 不做空指针算术
// ---------------------------------------------------------------------------
void test_span_empty()
{
    span<std::uint8_t> s;
    EXPECT(s.empty(), "默认 span 为空");
    EXPECT(s.begin() == nullptr, "begin() == nullptr");
    EXPECT(s.end() == nullptr, "end() 不得做 nullptr + 0");
    auto sub = s.subspan(0, 0);
    EXPECT(sub.empty(), "空 span 的 subspan 仍为空");
    EXPECT(sub.data() == nullptr, "subspan 不得做 nullptr + 0");
    auto first = s.first(10);
    EXPECT(first.empty(), "空 span 的 first(n) 仍为空");

    std::uint8_t arr[3] = {1, 2, 3};
    span<std::uint8_t> a(arr);
    EXPECT(a.end() == arr + 3, "非空 span 的 end 正常");
    EXPECT(a.subspan(1, 1).data() == arr + 1, "非空 span 的 subspan 正常");
}

// ---------------------------------------------------------------------------
// T4：PSM 的 CRC32 不再写 0
// ---------------------------------------------------------------------------
void test_psm_crc()
{
    std::vector<std::uint8_t> out;
    detail::build_psm(out, {{0x1B, 0xE0}, {0x90, 0xC0}}, 3);

    std::size_t i = 0;
    for (; i + 8 <= out.size(); ++i) {
        if (out[i] == 0 && out[i + 1] == 0 && out[i + 2] == 1 && out[i + 3] == 0xBC) {
            break;
        }
    }
    EXPECT(i + 8 <= out.size(), "应能找到 PSM 起始码");
    if (i + 8 > out.size()) { return; }

    const std::size_t body_len = (static_cast<std::size_t>(out[i + 4]) << 8) | out[i + 5];
    const std::size_t body_at  = i + 6;
    EXPECT(body_len >= 4 && body_at + body_len <= out.size(), "PSM 长度合法");
    if (body_len < 4 || body_at + body_len > out.size()) { return; }

    const std::uint32_t want = crc32_mpeg2_local(out.data() + body_at, body_len - 4);
    const std::size_t   pos  = body_at + body_len - 4;
    const std::uint32_t got  = (static_cast<std::uint32_t>(out[pos]) << 24) |
                               (static_cast<std::uint32_t>(out[pos + 1]) << 16) |
                               (static_cast<std::uint32_t>(out[pos + 2]) << 8) |
                               static_cast<std::uint32_t>(out[pos + 3]);
    EXPECT(got != 0, "PSM CRC 不应为 0");
    EXPECT(got == want, "PSM CRC 应与 MPEG-2 CRC32 一致");
}

// ---------------------------------------------------------------------------
// T5-a：TS 路径 dts 归一（无 DTS == pts，不泄漏 kNoTimestamp）
// ---------------------------------------------------------------------------
void test_ts_dts_normalized()
{
    StreamConfig s;
    s.codec = Codec::h264;
    const auto run = mux_write(Container::ts, {s}, 1, 90000, 90000);
    EXPECT(!run.ec, "TS 复用");
    EXPECT(!run.out.empty(), "TS 应有输出");

    DemuxerConfig cfg;
    cfg.container = Container::ts;
    std::error_code ec;
    int frames = 0;
    std::int64_t dts = 0;
    auto d = Demuxer::create(cfg, [&](const FrameInfo& fi, span<const std::uint8_t>) {
        ++frames;
        dts = fi.dts;
    }, nullptr, ec);
    if (!d) { EXPECT(false, "Demuxer 创建"); return; }
    d->feed(span<const std::uint8_t>(run.out.data(), run.out.size()));
    d->flush();
    EXPECT(frames == 1, "应解出 1 帧");
    EXPECT(dts == 90000, "无 DTS 时 fi.dts 应等于 pts（曾漏出 kNoTimestamp）");
}

// ---------------------------------------------------------------------------
// T5-b：AAC 拆分遇到非 ADTS 尾部要计 parse_errors，不能静默丢
// ---------------------------------------------------------------------------
void test_aac_tail_counted()
{
    // 一条合法 ADTS（frame_length=25）+ 20 字节垃圾尾巴
    std::vector<std::uint8_t> payload = {0xFF, 0xF1, 0x50, 0x80, 0x03, 0x3F, 0xFC};
    payload.resize(25, 0xAA);
    payload.resize(payload.size() + 20, 0xEE);

    StreamConfig s;
    s.codec    = Codec::aac;
    s.add_adts = false;   // 调用方自己给的 ADTS，库原样透传

    MuxerConfig mc;
    mc.container = Container::ts;
    std::vector<std::uint8_t> out;
    std::error_code ec;
    auto mx = Muxer::create(mc, [&out](span<const std::uint8_t> d) {
        out.insert(out.end(), d.begin(), d.end());
    }, ec);
    if (!mx) { EXPECT(false, "TS 复用器创建"); return; }
    auto sid = mx->addStream(s, ec);
    if (!sid) { EXPECT(false, "addStream"); return; }
    mx->write(*sid, FrameFlag::key, 0, 0, span<const std::uint8_t>(payload.data(), payload.size()));
    mx->flush();

    DemuxerConfig cfg;
    cfg.container = Container::ts;
    int frames = 0;
    auto d = Demuxer::create(cfg, [&](const FrameInfo&, span<const std::uint8_t>) {
        ++frames;
    }, nullptr, ec);
    if (!d) { EXPECT(false, "Demuxer 创建"); return; }
    d->feed(span<const std::uint8_t>(out.data(), out.size()));
    d->flush();

    EXPECT(frames == 1, "只应切出 1 个合法 ADTS 帧");
    EXPECT(d->stats().parse_errors >= 1, "非 ADTS 尾部应计入 parse_errors");
}

// ---------------------------------------------------------------------------
// T5-c：PS flush 残帧按真实流标注（不再硬编码 0xE0/视频）
// ---------------------------------------------------------------------------
void test_ps_flush_audio_residual()
{
    std::vector<std::uint8_t> buf;
    detail::build_pack_header(buf, 0, 80000);
    detail::build_psm(buf, {{0x90, 0xC0}}, 0);          // G.711A，stream_id 0xC0

    // 不定长 PES（length=0）：给头 + 30 字节数据，但没有下一个起始码 → 残帧
    buf.push_back(0x00); buf.push_back(0x00); buf.push_back(0x01); buf.push_back(0xC0);
    buf.push_back(0x00); buf.push_back(0x00);
    buf.push_back(0x80); buf.push_back(0x80);
    buf.push_back(0x05);
    put_timestamp(buf, 0x02, 90000);
    buf.resize(buf.size() + 30, 0xAB);

    DemuxerConfig cfg;
    cfg.container             = Container::ps;
    cfg.emit_partial_on_flush = true;
    std::error_code ec;
    int frames = 0;
    Codec codec = Codec::unknown;
    bool incomplete = false;
    auto d = Demuxer::create(cfg, [&](const FrameInfo& fi, span<const std::uint8_t>) {
        ++frames;
        codec      = fi.codec;
        incomplete = hasFlag(fi.flags, FrameFlag::incomplete);
    }, nullptr, ec);
    if (!d) { EXPECT(false, "Demuxer 创建"); return; }
    d->feed(span<const std::uint8_t>(buf.data(), buf.size()));
    d->flush();

    EXPECT(frames == 1, "应吐出 1 条残帧");
    EXPECT(codec == Codec::g711a, "残帧应按 PSM 里的音频流标注（曾硬编码成视频 H.264）");
    EXPECT(incomplete, "残帧应带 incomplete 标志");
}

// ---------------------------------------------------------------------------
// T6：addStream 容量 / 重复 / 保留 PID
// ---------------------------------------------------------------------------
void test_addstream_validation()
{
    // PS 容量 32
    {
        MuxerConfig mc;
        mc.container = Container::ps;
        std::error_code ec;
        auto mx = Muxer::create(mc, [](span<const std::uint8_t>) {}, ec);
        if (!mx) { EXPECT(false, "PS 创建"); return; }
        StreamConfig s;
        s.codec = Codec::g711a;   // 音频区间 0xC0..0xDF，正好 32 个
        bool ok = true;
        for (int i = 0; i < 32; ++i) {
            if (!mx->addStream(s, ec)) { ok = false; break; }
        }
        EXPECT(ok, "PS 前 32 条音频流应成功");
        const auto id = mx->addStream(s, ec);
        EXPECT(!id.has_value(), "PS 第 33 条流应失败");
        EXPECT(ec == MpegErrc::capacity_exceeded, "PS 超限应返回 capacity_exceeded");
    }

    // PS 重复 stream_id
    {
        MuxerConfig mc;
        mc.container = Container::ps;
        std::error_code ec;
        auto mx = Muxer::create(mc, [](span<const std::uint8_t>) {}, ec);
        StreamConfig a;
        a.codec     = Codec::h264;
        a.stream_id = 0xE1;
        EXPECT(mx->addStream(a, ec).has_value(), "PS 首条流");
        const auto dup = mx->addStream(a, ec);
        EXPECT(!dup.has_value(), "重复 stream_id 应失败");
        EXPECT(ec == MpegErrc::invalid_argument, "重复 stream_id 应返回 invalid_argument");
    }

    // TS 保留 PID / 重复 / 容量 16
    {
        MuxerConfig mc;
        mc.container = Container::ts;
        std::error_code ec;
        auto mx = Muxer::create(mc, [](span<const std::uint8_t>) {}, ec);
        StreamConfig bad;
        bad.codec = Codec::h264;
        bad.pid   = 0x1FFF;                                    // NULL PID
        EXPECT(!mx->addStream(bad, ec).has_value(), "NULL PID 应被拒绝");
        bad.pid = 0x1000;                                      // PMT PID（默认）
        EXPECT(!mx->addStream(bad, ec).has_value(), "PMT PID 应被拒绝");

        StreamConfig good;
        good.codec = Codec::h264;
        good.pid   = 0x0101;
        EXPECT(mx->addStream(good, ec).has_value(), "正常 PID 应成功");
        EXPECT(!mx->addStream(good, ec).has_value(), "重复 PID 应失败");
        EXPECT(ec == MpegErrc::invalid_argument, "重复 PID 应返回 invalid_argument");
    }

    {
        MuxerConfig mc;
        mc.container = Container::ts;
        std::error_code ec;
        auto mx = Muxer::create(mc, [](span<const std::uint8_t>) {}, ec);
        bool ok = true;
        for (int i = 0; i < 16; ++i) {
            StreamConfig s;
            s.codec = Codec::h264;
            s.pid   = static_cast<std::uint16_t>(0x0100 + i);
            if (!mx->addStream(s, ec)) { ok = false; break; }
        }
        EXPECT(ok, "TS 前 16 条流应成功");
        StreamConfig s;
        s.codec = Codec::h264;
        s.pid   = 0x0120;
        EXPECT(!mx->addStream(s, ec).has_value(), "TS 第 17 条流应失败");
        EXPECT(ec == MpegErrc::capacity_exceeded, "TS 超限应返回 capacity_exceeded");
    }
}

} // namespace

int main()
{
    std::printf("=== mpegmech P2 回归用例 ===\n");
    test_psi_version_increments();
    test_span_empty();
    test_psm_crc();
    test_ts_dts_normalized();
    test_aac_tail_counted();
    test_ps_flush_audio_residual();
    test_addstream_validation();
    if (g_fail == 0) {
        std::printf("全部通过\n");
        return 0;
    }
    std::printf("%d 项失败\n", g_fail);
    return 1;
}
