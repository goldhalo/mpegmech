// =============================================================================
//  test_p3_regressions.cpp —— 第三批 P2 修复的回归用例（全部自包含）
//
//  覆盖：
//    T7  TS 解封装校验 PAT/PMT CRC；PS 解封装校验 PSM CRC
//    T8  max_buffered_bytes == 0 表示"不限制"
//    T9  PS 取消按 stream_id 猜 codec（缺 PSM 时返回 unknown）
//    T10 TS 包长常量去重（两个内部头可同时包含）
//    T11 PS 自动 stream_id 超范围时返回 capacity_exceeded
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/muxer.hpp>

#include "ps_muxer.hpp"
#include "ts_demuxer.hpp"
#include "ts_muxer.hpp"

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

void append_crc(std::vector<std::uint8_t>& sec)
{
    const std::uint32_t c = crc32_mpeg2_local(sec.data(), sec.size());
    sec.push_back(static_cast<std::uint8_t>(c >> 24));
    sec.push_back(static_cast<std::uint8_t>((c >> 16) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>((c >> 8) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>(c & 0xFF));
}

void append_packet(std::vector<std::uint8_t>& out, std::uint16_t pid, bool pusi,
                   std::uint8_t cc, const std::uint8_t* payload, std::size_t n)
{
    out.push_back(0x47);
    out.push_back(static_cast<std::uint8_t>((pusi ? 0x40 : 0x00) | ((pid >> 8) & 0x1F)));
    out.push_back(static_cast<std::uint8_t>(pid & 0xFF));
    out.push_back(static_cast<std::uint8_t>(0x10 | (cc & 0x0F)));
    if (n != 0) { out.insert(out.end(), payload, payload + n); }
    out.insert(out.end(), 184 - n, 0xFF);
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

// 构造一段 PS：PackHeader + PSM（可选损坏 CRC / 可选缺失）+ 一个 PES 帧
std::vector<std::uint8_t> make_ps(bool with_psm, bool corrupt_psm_crc, bool complete)
{
    std::vector<std::uint8_t> buf;
    detail::build_pack_header(buf, 0, 80000);
    if (with_psm) {
        std::vector<std::uint8_t> psm;
        detail::build_psm(psm, {{0x90, 0xC0}}, 0);   // G.711A
        if (corrupt_psm_crc) { psm.back() ^= 0xFF; }
        buf.insert(buf.end(), psm.begin(), psm.end());
    }
    const std::uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    detail::build_pes_header(buf, 0xC0, 90000, 90000, sizeof payload,
                             /*explicit_len=*/complete);
    buf.insert(buf.end(), payload, payload + sizeof payload);
    return buf;
}

/// 跑 PS 解封装，返回第一帧的 codec / 错误码。
struct PsResult {
    std::error_code ec;
    int             frames = 0;
    Codec           codec  = Codec::unknown;
};

PsResult demux_ps(const std::vector<std::uint8_t>& buf, DemuxerConfig cfg)
{
    PsResult r;
    cfg.container = Container::ps;
    auto d = Demuxer::create(cfg, [&r](const FrameInfo& fi, span<const std::uint8_t>) {
        ++r.frames;
        r.codec = fi.codec;
    }, nullptr, r.ec);
    if (!d) { return r; }
    r.ec = d->feed(span<const std::uint8_t>(buf.data(), buf.size()));
    if (!r.ec) { d->flush(); }
    return r;
}

// ---------------------------------------------------------------------------
// T7-a：坏 CRC 的 PAT 必须被拒绝（不能注册任何流）
// ---------------------------------------------------------------------------
void test_pat_bad_crc_rejected()
{
    std::vector<std::uint8_t> pat_sec = {0x00, 0xB0, 0x0D, 0x00, 0x01, 0xC1,
                                         0x00, 0x00, 0x00, 0x01, 0xF0, 0x00};
    append_crc(pat_sec);
    pat_sec.back() ^= 0xFF;   // 故意破坏 CRC

    std::vector<std::uint8_t> pat_payload(1, 0x00);
    pat_payload.insert(pat_payload.end(), pat_sec.begin(), pat_sec.end());

    std::vector<std::uint8_t> pmt_sec = {0x02, 0xB0, 0x12, 0x00, 0x01, 0xC1,
                                         0x00, 0x00, 0xE1, 0x00, 0xF0, 0x00,
                                         0x1B, 0xE1, 0x00, 0xF0, 0x00};
    append_crc(pmt_sec);
    std::vector<std::uint8_t> pmt_payload(1, 0x00);
    pmt_payload.insert(pmt_payload.end(), pmt_sec.begin(), pmt_sec.end());

    std::vector<std::uint8_t> ts;
    append_packet(ts, 0x0000, true, 0, pat_payload.data(), pat_payload.size());
    append_packet(ts, 0x1000, true, 0, pmt_payload.data(), pmt_payload.size());
    append_packet(ts, 0x1FFF, false, 0, nullptr, 0);

    detail::TsDemuxer demux;
    std::vector<detail::TsDemuxer::Frame> frames;
    demux.feed(ts.data(), ts.size(), frames);
    EXPECT(demux.streams().empty(), "坏 CRC 的 PAT 不应注册任何节目/流");
    EXPECT(demux.bad_packets() >= 1, "坏 CRC 应计入 bad_packets");
}

// ---------------------------------------------------------------------------
// T7-b：坏 CRC 的 PMT 必须被拒绝
// ---------------------------------------------------------------------------
void test_pmt_bad_crc_rejected()
{
    std::vector<std::uint8_t> pat_sec = {0x00, 0xB0, 0x0D, 0x00, 0x01, 0xC1,
                                         0x00, 0x00, 0x00, 0x01, 0xF0, 0x00};
    append_crc(pat_sec);
    std::vector<std::uint8_t> pat_payload(1, 0x00);
    pat_payload.insert(pat_payload.end(), pat_sec.begin(), pat_sec.end());

    std::vector<std::uint8_t> pmt_sec = {0x02, 0xB0, 0x12, 0x00, 0x01, 0xC1,
                                         0x00, 0x00, 0xE1, 0x00, 0xF0, 0x00,
                                         0x1B, 0xE1, 0x00, 0xF0, 0x00};
    append_crc(pmt_sec);
    pmt_sec.back() ^= 0xFF;

    std::vector<std::uint8_t> pmt_payload(1, 0x00);
    pmt_payload.insert(pmt_payload.end(), pmt_sec.begin(), pmt_sec.end());

    std::vector<std::uint8_t> ts;
    append_packet(ts, 0x0000, true, 0, pat_payload.data(), pat_payload.size());
    append_packet(ts, 0x1000, true, 0, pmt_payload.data(), pmt_payload.size());
    append_packet(ts, 0x1FFF, false, 0, nullptr, 0);

    detail::TsDemuxer demux;
    std::vector<detail::TsDemuxer::Frame> frames;
    demux.feed(ts.data(), ts.size(), frames);
    EXPECT(demux.streams().empty(), "坏 CRC 的 PMT 不应注册任何 ES");
}

// ---------------------------------------------------------------------------
// T7-c：坏 CRC 的 PSM 必须被拒绝（PS 路径）
// ---------------------------------------------------------------------------
void test_psm_bad_crc_rejected()
{
    const auto r = demux_ps(make_ps(/*with_psm=*/true, /*corrupt=*/true, true),
                            DemuxerConfig{});
    EXPECT(r.ec.value() == 0, "feed 不应报错");
    EXPECT(r.frames == 1, "应解出 1 帧");
    EXPECT(r.codec == Codec::unknown,
           "坏 CRC 的 PSM 应被忽略（当初会按 PSM 认成 G.711A）");
}

// ---------------------------------------------------------------------------
// T7-d：合法 PSM 仍能被识别（防止 CRC 校验误杀）
// ---------------------------------------------------------------------------
void test_psm_good_crc_accepted()
{
    const auto r = demux_ps(make_ps(/*with_psm=*/true, /*corrupt=*/false, true),
                            DemuxerConfig{});
    EXPECT(r.frames == 1, "应解出 1 帧");
    EXPECT(r.codec == Codec::g711a, "合法 PSM 应把流识别成 G.711A");
}

// ---------------------------------------------------------------------------
// T9：缺 PSM 时不再按 stream_id 猜 codec
// ---------------------------------------------------------------------------
void test_codec_fallback_unknown()
{
    const auto r = demux_ps(make_ps(/*with_psm=*/false, false, true), DemuxerConfig{});
    EXPECT(r.frames == 1, "应解出 1 帧");
    EXPECT(r.codec == Codec::unknown,
           "缺 PSM 时 codec 应为 unknown（曾把 0xE0 猜成 H.264、0xC0 猜成 AAC）");
}

// ---------------------------------------------------------------------------
// T8：max_buffered_bytes == 0 表示不限制
// ---------------------------------------------------------------------------
void test_max_buffered_zero_means_unlimited()
{
    std::vector<std::uint8_t> buf;
    detail::build_pack_header(buf, 0, 80000);
    // 不定长 PES：5 MiB 数据但没有下一个起始码 → 会一直留在内部缓冲
    buf.push_back(0x00); buf.push_back(0x00); buf.push_back(0x01); buf.push_back(0xE0);
    buf.push_back(0x00); buf.push_back(0x00);
    buf.push_back(0x80); buf.push_back(0x80);
    buf.push_back(0x05);
    put_timestamp(buf, 0x02, 90000);
    buf.resize(buf.size() + 5u * 1024u * 1024u, 0xAB);

    DemuxerConfig cfg;
    cfg.container          = Container::ps;
    cfg.max_buffered_bytes = 0;   // 不限制
    std::error_code ec;
    auto d = Demuxer::create(cfg, [](const FrameInfo&, span<const std::uint8_t>) {},
                             nullptr, ec);
    if (!d) { EXPECT(false, "Demuxer 创建"); return; }
    const auto fe = d->feed(span<const std::uint8_t>(buf.data(), buf.size()));
    EXPECT(!fe, "max_buffered_bytes=0 时不应触发 buffer_overflow");
    EXPECT(d->buffered() >= 5u * 1024u * 1024u, "不限制时数据应留在内部缓冲");
    d->reset();
}

// ---------------------------------------------------------------------------
// T11：PS 自动 stream_id 用尽（视频 0xE0..0xEF / 音频 0xC0..0xDF）
// ---------------------------------------------------------------------------
void test_ps_streamid_exhausted()
{
    MuxerConfig mc;
    mc.container = Container::ps;
    std::error_code ec;
    auto mx = Muxer::create(mc, [](span<const std::uint8_t>) {}, ec);
    if (!mx) { EXPECT(false, "PS 复用器创建"); return; }

    StreamConfig video;
    video.codec = Codec::h264;
    for (int i = 0; i < 16; ++i) {
        EXPECT(mx->addStream(video, ec).has_value(), "PS 前 16 条视频流应成功");
    }
    const auto id = mx->addStream(video, ec);
    EXPECT(!id.has_value(), "第 17 条视频流应失败（stream_id 区间用尽）");
    EXPECT(ec == MpegErrc::capacity_exceeded, "应返回 capacity_exceeded");
}

} // namespace

int main()
{
    std::printf("=== mpegmech P3 回归用例 ===\n");
    test_pat_bad_crc_rejected();
    test_pmt_bad_crc_rejected();
    test_psm_bad_crc_rejected();
    test_psm_good_crc_accepted();
    test_codec_fallback_unknown();
    test_max_buffered_zero_means_unlimited();
    test_ps_streamid_exhausted();
    if (g_fail == 0) {
        std::printf("全部通过\n");
        return 0;
    }
    std::printf("%d 项失败\n", g_fail);
    return 1;
}
