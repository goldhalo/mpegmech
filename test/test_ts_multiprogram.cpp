// =============================================================================
//  test_ts_multiprogram.cpp —— 多节目 TS
// =============================================================================
//  一个 TS 里可以有多套节目，每套节目有自己的 PMT（不同 PID）：
//
//      PAT ──┬─> program 1 → PMT@0x1000 → ES 0x100 (H.264)
//            └─> program 2 → PMT@0x1001 → ES 0x200 (AAC)
//
//  早先的实现只记住 PAT 里**最后一个** PMT PID，多节目流等于只解最后一套，
//  而且 FrameInfo.program 恒填 1 —— 下游根本没法按节目区分。
//
//  这个用例手工拼流（我们自己的复用器只做单节目），顺带把
//  "PAT/PMT 里哪些字段在哪个偏移" 这件事钉死在测试里。
// =============================================================================
#include <mpegmech/demuxer.hpp>

#include "ps_muxer.hpp"     // detail::build_pes_header
#include "ts_muxer.hpp"     // detail::build_pat / build_pmt / write_ts_packets

#include <cstdio>
#include <map>
#include <vector>

using namespace mpegmech;

namespace {

int g_fail = 0;
#define EXPECT(c, m) do { if (!(c)) { std::printf("  [FAIL] %s (line %d)\n", (m), __LINE__); ++g_fail; } } while (0)

using Bytes = std::vector<std::uint8_t>;

/// 造一段假视频帧（以起始码开头的任意字节，够长能跨多个 188 包）。
Bytes fake_video(std::size_t seed, std::size_t len)
{
    Bytes b;
    b.reserve(len + 8);
    const std::uint8_t nal[] = {0x00, 0x00, 0x00, 0x01, 0x65};   // IDR
    b.insert(b.end(), nal, nal + sizeof(nal));
    for (std::size_t i = 0; i < len; ++i) {
        b.push_back(static_cast<std::uint8_t>((seed * 31 + i * 7) & 0xFF));
    }
    return b;
}

/// 造一帧 ADTS 打包的 AAC（7 字节头 + 任意载荷）。
Bytes fake_aac(std::size_t seed, std::size_t payload)
{
    const std::size_t flen = payload + 7;
    Bytes             b;
    b.reserve(flen);
    b.push_back(0xFF);
    b.push_back(0xF1);                                    // MPEG-4, no CRC
    b.push_back(0x50);                                    // profile/采样率索引/声道
    b.push_back(static_cast<std::uint8_t>(0x80 | ((flen >> 11) & 0x03)));
    b.push_back(static_cast<std::uint8_t>((flen >> 3) & 0xFF));
    b.push_back(static_cast<std::uint8_t>(((flen & 0x07) << 5) | 0x1F));
    b.push_back(0xFC);                                    // buffer fullness + 1 frame
    for (std::size_t i = 0; i < payload; ++i) {
        b.push_back(static_cast<std::uint8_t>((seed * 13 + i) & 0xFF));
    }
    return b;
}

Bytes make_pes(std::uint8_t stream_id, std::int64_t pts, const Bytes& es)
{
    Bytes pes;
    detail::build_pes_header(pes, stream_id, pts, pts, es.size(), true);
    pes.insert(pes.end(), es.begin(), es.end());
    return pes;
}

struct Sniffed {
    int           stream_id = -1;
    Codec         codec     = Codec::unknown;
    std::uint16_t program   = 0;
};

} // namespace

int main()
{
    std::printf("=== 多节目 TS ===\n");

    constexpr std::uint16_t kProg1 = 1;      // 显式给 uint16_t，避免 pair 里
    constexpr std::uint16_t kProg2 = 2;      // int → uint16_t 触发 C4244
    constexpr std::uint16_t kPmt1 = 0x1000;
    constexpr std::uint16_t kPmt2 = 0x1001;
    constexpr std::uint16_t kVid  = 0x0100;
    constexpr std::uint16_t kAud  = 0x0200;

    Bytes           ts;
    std::uint8_t    cc_pat = 0, cc_pmt1 = 0, cc_pmt2 = 0, cc_v = 0, cc_a = 0;
    constexpr std::size_t kFrames = 6;

    // ---- PAT：两套节目 ----
    detail::build_pat(ts, 1, {{kProg1, kPmt1}, {kProg2, kPmt2}}, 0, cc_pat);

    // ---- PMT：节目 1 是 H.264 视频，节目 2 是 AAC 音频 ----
    detail::build_pmt(ts, kProg1, kPmt1, kVid, {{0x1B, kVid}}, 0, cc_pmt1);
    detail::build_pmt(ts, kProg2, kPmt2, 0, {{0x0F, kAud}}, 0, cc_pmt2);

    // ---- ES：两路交错写 ----
    std::vector<Bytes> src_v, src_a;
    for (std::size_t i = 0; i < kFrames; ++i) {
        src_v.push_back(fake_video(i, 300 + i * 40));
        src_a.push_back(fake_aac(i, 100 + i * 5));

        const auto pv = make_pes(0xE0, static_cast<std::int64_t>(i) * 3600, src_v.back());
        const auto pa = make_pes(0xC0, static_cast<std::int64_t>(i) * 2089, src_a.back());

        detail::write_ts_packets(ts, kVid, pv.data(), pv.size(), true, cc_v);
        detail::write_ts_packets(ts, kAud, pa.data(), pa.size(), true, cc_a);

        // 中途重发一次 PSI（真实流每 100ms 一次）
        if (i == 2) {
            detail::build_pat(ts, 1, {{kProg1, kPmt1}, {kProg2, kPmt2}}, 0, cc_pat);
            detail::build_pmt(ts, kProg1, kPmt1, kVid, {{0x1B, kVid}}, 0, cc_pmt1);
            detail::build_pmt(ts, kProg2, kPmt2, 0, {{0x0F, kAud}}, 0, cc_pmt2);
        }
    }

    std::printf("手工拼出 %zu 字节（%zu 个包）\n", ts.size(), ts.size() / 188);
    EXPECT(ts.size() % 188 == 0, "整包");

    // ---- 解 ----
    std::map<int, Sniffed>           info;
    std::map<int, std::vector<Bytes>> got;
    std::vector<std::uint16_t>       frame_program;

    std::error_code ec;
    std::size_t     stream_cbs = 0;

    DemuxerConfig dc;
    dc.container = Container::unknown;
    auto dx = Demuxer::create(dc,
        [&](const FrameInfo& fi, span<const std::uint8_t> payload) {
            got[fi.stream_id].emplace_back(payload.begin(), payload.end());
            frame_program.push_back(fi.program);
        },
        [&](const StreamInfo& si) {
            ++stream_cbs;
            info[si.stream_id] = Sniffed{si.stream_id, si.codec, si.program};
        }, ec);
    EXPECT(dx != nullptr, "创建解封装器");

    // 块长 97，故意不对齐
    std::size_t i = 0;
    while (i < ts.size()) {
        std::size_t n = 97;
        if (n > ts.size() - i) { n = ts.size() - i; }
        dx->feed(span<const std::uint8_t>(ts.data() + i, n));
        i += n;
    }
    dx->flush();

    const auto& st = dx->stats();
    std::printf("流回调 %zu 次，解出 %zu 条流，resync=%llu err=%llu\n",
                stream_cbs, got.size(),
                static_cast<unsigned long long>(st.resyncs),
                static_cast<unsigned long long>(st.parse_errors));

    EXPECT(st.resyncs == 0, "无误同步");
    EXPECT(st.parse_errors == 0, "无结构错误");
    EXPECT(got.size() == 2, "解出两条流");

    // ---- 按 codec 找到两条流，核对 program / 内容 ----
    const Sniffed* v = nullptr;
    const Sniffed* a = nullptr;
    for (const auto& kv : info) {
        if (kv.second.codec == Codec::h264) { v = &kv.second; }
        if (kv.second.codec == Codec::aac)  { a = &kv.second; }
    }
    EXPECT(v != nullptr, "认出 H.264");
    EXPECT(a != nullptr, "认出 AAC");

    if (v != nullptr) {
        std::printf("  视频：stream_id=%d program=%u pid 无关，%zu 帧\n",
                    v->stream_id, v->program, got[v->stream_id].size());
        EXPECT(v->program == kProg1, "视频属于节目 1");
        EXPECT(got[v->stream_id] == src_v, "视频逐字节一致");
    }
    if (a != nullptr) {
        std::printf("  音频：stream_id=%d program=%u，%zu 帧\n",
                    a->stream_id, a->program, got[a->stream_id].size());
        EXPECT(a->program == kProg2, "音频属于节目 2");
        EXPECT(got[a->stream_id] == src_a, "音频逐字节一致（ADTS 未拆坏）");
    }

    // ---- 每帧的 program 也必须对 ----
    std::size_t bad = 0;
    for (std::size_t k = 0; k < frame_program.size(); ++k) {
        // 前 kFrames 帧是视频（program 1），后面交错的是音频（program 2）
        if (frame_program[k] != 1 && frame_program[k] != 2) { ++bad; }
    }
    EXPECT(bad == 0, "每帧都带合法 program");

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
