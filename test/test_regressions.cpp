// =============================================================================
//  test_regressions.cpp —— 2026-11-07 审查后补的回归用例（全部自包含）
//
//  覆盖：
//    P1-1 PS：PackHeader 后紧跟 PES（无 SystemHeader/PSM）不能丢帧
//    P1-2 PS：PES 的 DTS 必须解析出来（不能被 PTS 覆盖）
//    P1-3 setHandler(nullptr) 后 feed 不得抛 bad_function_call
//    P1-4 H.265 无 AUD 时按 first_slice_segment_in_pic_flag 切帧
//    P1-5 嗅探失败后继续 feed 不得让内部缓冲无界增长
//    P2-1 TS 失步重同步后，未完成的 PES 必须丢弃（不产坏帧）
//
//  不依赖 /tmp 下的任何素材文件，CI 里能真实执行。
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include "ps_muxer.hpp"
#include "ts_demuxer.hpp"

#include <algorithm>
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

/// MPEG-2 CRC32（与库内实现同参数），用于给手拼的 PAT/PMT 补 CRC。
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

/// 用本库复用器产出一段 PS（给 P1-2 / P1-3 用）。
std::vector<std::uint8_t> mux_ps(int frames, std::int64_t pts, std::int64_t dts)
{
    MuxerConfig mc;
    mc.container = Container::ps;
    std::vector<std::uint8_t> out;
    std::error_code ec;
    auto mx = Muxer::create(mc, [&out](span<const std::uint8_t> d) {
        out.insert(out.end(), d.begin(), d.end());
    }, ec);
    if (!mx) { return out; }
    StreamConfig sc;
    sc.codec = Codec::h264;
    const auto sid = mx->addStream(sc, ec);
    if (!sid) { return out; }
    for (int i = 0; i < frames; ++i) {
        mx->write(*sid, i == 0 ? FrameFlag::key : FrameFlag::none,
                  pts + i * 3000, dts + i * 3000,
                  span<const std::uint8_t>(kFrame264, sizeof kFrame264));
    }
    mx->flush();
    return out;
}

// ---------------------------------------------------------------------------
// P1-1：PackHeader（14 字节）+ PES，中间没有 SystemHeader/PSM
// ---------------------------------------------------------------------------
void test_ps_pack_header_followed_by_pes()
{
    std::vector<std::uint8_t> buf;
    detail::build_pack_header(buf, 0, 80000);
    const std::uint8_t payload[] = {0x00, 0x00, 0x00, 0x01, 0x65, 0x11, 0x22, 0x33};
    detail::build_pes_header(buf, 0xE0, 90000, 90000, sizeof payload,
                             /*explicit_len=*/true);
    buf.insert(buf.end(), payload, payload + sizeof payload);

    DemuxerConfig cfg;
    cfg.container = Container::ps;
    std::error_code ec;
    int frames = 0;
    bool payload_ok = false;
    auto d = Demuxer::create(cfg, [&](const FrameInfo&, span<const std::uint8_t> es) {
        ++frames;
        payload_ok = es.size() == sizeof payload &&
                     std::equal(es.begin(), es.end(), payload);
    }, nullptr, ec);
    EXPECT(d != nullptr, "Demuxer 创建");
    if (!d) { return; }

    const auto fe = d->feed(span<const std::uint8_t>(buf.data(), buf.size()));
    d->flush();
    EXPECT(!fe, "feed 不应报错");
    EXPECT(frames == 1, "PH+PES 应解出 1 帧（曾经因多跳 4 字节解出 0 帧）");
    EXPECT(payload_ok, "解出的载荷应与写入逐字节一致");
}

// ---------------------------------------------------------------------------
// P1-2：PES 里 PTS != DTS 时 DTS 不能丢
// ---------------------------------------------------------------------------
void test_ps_pes_dts()
{
    const auto ps = mux_ps(1, 90000, 45000);
    EXPECT(!ps.empty(), "复用器应产出 PS 数据");

    DemuxerConfig cfg;
    cfg.container = Container::ps;
    std::error_code ec;
    int frames = 0;
    std::int64_t pts = 0, dts = 0;
    auto d = Demuxer::create(cfg, [&](const FrameInfo& fi, span<const std::uint8_t>) {
        ++frames;
        pts = fi.pts;
        dts = fi.dts;
    }, nullptr, ec);
    EXPECT(d != nullptr, "Demuxer 创建");
    if (!d) { return; }

    d->feed(span<const std::uint8_t>(ps.data(), ps.size()));
    d->flush();
    EXPECT(frames == 1, "应解出 1 帧");
    EXPECT(pts == 90000, "PTS 应为 90000");
    EXPECT(dts == 45000, "DTS 必须保留（曾经被当成无 DTS，回落成 pts）");
}

// ---------------------------------------------------------------------------
// P1-3：setHandler(nullptr) 后 feed 仍消费数据、不回调、不抛异常
// ---------------------------------------------------------------------------
void test_set_handler_null()
{
    const auto ps = mux_ps(2, 90000, 90000);
    EXPECT(!ps.empty(), "复用器应产出 PS 数据");

    DemuxerConfig cfg;
    cfg.container = Container::ps;
    std::error_code ec;
    int frames = 0;
    auto d = Demuxer::create(cfg, [&](const FrameInfo&, span<const std::uint8_t>) {
        ++frames;
    }, nullptr, ec);
    EXPECT(d != nullptr, "Demuxer 创建");
    if (!d) { return; }

    d->feed(span<const std::uint8_t>(ps.data(), ps.size()));
    const int before = frames;

    d->setHandler(nullptr);
    bool threw = false;
    try {
        d->feed(span<const std::uint8_t>(ps.data(), ps.size()));
        d->flush();
    } catch (...) {
        threw = true;
    }
    EXPECT(!threw, "setHandler(nullptr) 后 feed 不得抛异常");
    EXPECT(frames == before, "解绑后不应再收到回调");
}

// ---------------------------------------------------------------------------
// P1-4：H.265 无 AUD 时按 first_slice_segment_in_pic_flag 切帧
// ---------------------------------------------------------------------------
void test_h265_split_without_aud()
{
    // 两帧、无 AUD；NAL 头 2 字节（0x26,0x01），slice header 首字节 0x80
    //     bit7 = first_slice_segment_in_pic_flag = 1
    const std::uint8_t f1[] = {0x00, 0x00, 0x01, 0x26, 0x01, 0x80, 0xAA, 0xBB};
    const std::uint8_t f2[] = {0x00, 0x00, 0x01, 0x26, 0x01, 0x80, 0xCC, 0xDD};
    std::vector<std::uint8_t> stream;
    stream.insert(stream.end(), f1, f1 + sizeof f1);
    stream.insert(stream.end(), f2, f2 + sizeof f2);

    EsSplitter::Config cfg;
    cfg.codec = Codec::h265;
    std::error_code ec;
    auto sp = EsSplitter::create(cfg, ec);
    EXPECT(sp != nullptr, "EsSplitter 创建");
    if (!sp) { return; }

    int frames = 0;
    sp->feed(span<const std::uint8_t>(stream.data(), stream.size()),
             [&frames](span<const std::uint8_t>) { ++frames; });
    sp->flush([&frames](span<const std::uint8_t>) { ++frames; });
    EXPECT(frames == 2, "H.265 两帧（无 AUD）应切出 2 帧（曾经只有 1 帧）");
}

// ---------------------------------------------------------------------------
// P1-5：嗅探失败后继续 feed，内部缓冲必须有界
// ---------------------------------------------------------------------------
void test_unsupported_container_bounded()
{
    DemuxerConfig cfg;   // container = unknown → 自动嗅探
    std::error_code ec;
    auto d = Demuxer::create(cfg, [](const FrameInfo&, span<const std::uint8_t>) {},
                             nullptr, ec);
    EXPECT(d != nullptr, "Demuxer 创建");
    if (!d) { return; }

    std::vector<std::uint8_t> junk(2000, 0x55);
    const auto e1 = d->feed(span<const std::uint8_t>(junk.data(), junk.size()));
    EXPECT(e1 == MpegErrc::unsupported_container, "垃圾数据应报 unsupported_container");

    for (int i = 0; i < 500; ++i) {
        std::vector<std::uint8_t> chunk(512, 0xAA);
        d->feed(span<const std::uint8_t>(chunk.data(), chunk.size()));
    }
    // 有界：最多保留一个嗅探窗口（1 KiB）多一点
    EXPECT(d->buffered() <= 2048, "继续喂垃圾时内部缓冲必须有界（曾经无界增长）");
}

// ---------------------------------------------------------------------------
// P2-1：TS 失步重同步后，未完成 PES 必须丢弃
// ---------------------------------------------------------------------------
namespace {

void append_packet(std::vector<std::uint8_t>& out, std::uint16_t pid, bool pusi,
                   std::uint8_t cc, const std::uint8_t* payload, std::size_t n)
{
    out.push_back(0x47);
    out.push_back(static_cast<std::uint8_t>((pusi ? 0x40 : 0x00) | ((pid >> 8) & 0x1F)));
    out.push_back(static_cast<std::uint8_t>(pid & 0xFF));
    out.push_back(static_cast<std::uint8_t>(0x10 | (cc & 0x0F)));
    if (n != 0) {
        out.insert(out.end(), payload, payload + n);
    }
    out.insert(out.end(), 184 - n, 0xFF);
}

} // namespace

void test_ts_resync_drops_partial_pes()
{
    // PAT：program 1 → PMT PID 0x1000
    std::vector<std::uint8_t> pat_sec = {
        0x00, 0xB0, 0x0D, 0x00, 0x01, 0xC1, 0x00, 0x00,
        0x00, 0x01, 0xF0, 0x00,
    };
    append_crc(pat_sec);
    std::vector<std::uint8_t> pat_payload(1, 0x00);   // pointer_field
    pat_payload.insert(pat_payload.end(), pat_sec.begin(), pat_sec.end());

    // PMT：program 1 下一条 H.264 ES，PID 0x0100，PCR_PID 0x0100
    std::vector<std::uint8_t> pmt_sec = {
        0x02, 0xB0, 0x12, 0x00, 0x01, 0xC1, 0x00, 0x00,
        0xE1, 0x00, 0xF0, 0x00,
        0x1B, 0xE1, 0x00, 0xF0, 0x00,
    };
    append_crc(pmt_sec);
    std::vector<std::uint8_t> pmt_payload(1, 0x00);
    pmt_payload.insert(pmt_payload.end(), pmt_sec.begin(), pmt_sec.end());

    // 一条不定长 PES（长度字段=0）：先给两包数据，但不关闭
    std::vector<std::uint8_t> pes;
    const std::uint8_t hdr[] = {0x00, 0x00, 0x01, 0xE0, 0x00, 0x00,
                                0x80, 0x80, 0x05, 0x21, 0x00, 0x01, 0x00, 0x01};
    pes.insert(pes.end(), hdr, hdr + sizeof hdr);
    pes.resize(pes.size() + 400, 0xAB);

    std::vector<std::uint8_t> data;
    append_packet(data, 0x0100, true, 0, pes.data(), 184);
    append_packet(data, 0x0100, false, 1, pes.data() + 184, 184);

    // 一段坏同步数据：首字节 0x00 + 3 个合法包 → 触发"失步后重新同步"
    std::vector<std::uint8_t> bad;
    bad.push_back(0x00);
    append_packet(bad, 0x1FFF, false, 0, nullptr, 0);
    append_packet(bad, 0x1FFF, false, 0, nullptr, 0);
    append_packet(bad, 0x1FFF, false, 0, nullptr, 0);

    std::vector<std::uint8_t> psi;
    append_packet(psi, 0x0000, true, 0, pat_payload.data(), pat_payload.size());
    append_packet(psi, 0x1000, true, 0, pmt_payload.data(), pmt_payload.size());
    // 初始同步要求连续 3 个 0x47（间隔 188），补一个 NULL 包
    append_packet(psi, 0x1FFF, false, 0, nullptr, 0);

    detail::TsDemuxer ts;
    std::vector<detail::TsDemuxer::Frame> out;
    ts.feed(psi.data(), psi.size(), out);
    EXPECT(ts.streams().size() == 1u, "PMT 应登记 1 条 ES");

    ts.feed(data.data(), data.size(), out);
    EXPECT(ts.buffered_bytes() > 0, "失步前应有未完成 PES 累积");

    ts.feed(bad.data(), bad.size(), out);
    EXPECT(ts.buffered_bytes() == 0, "重同步后必须丢弃未完成 PES（曾经保留旧累积）");
}

} // namespace

int main()
{
    std::printf("=== mpegmech 回归用例 ===\n");
    test_ps_pack_header_followed_by_pes();
    test_ps_pes_dts();
    test_set_handler_null();
    test_h265_split_without_aud();
    test_unsupported_container_bounded();
    test_ts_resync_drops_partial_pes();
    if (g_fail == 0) {
        std::printf("全部通过\n");
        return 0;
    }
    std::printf("%d 项失败\n", g_fail);
    return 1;
}
