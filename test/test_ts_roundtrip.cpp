// =============================================================================
//  test_ts_roundtrip.cpp —— H264/H265/AV → TS → 解回，逐字节比对
// =============================================================================
//  这个测试专门盯 TS 路径上最容易错的三件事：
//    1. PES 分片重组（一帧跨多个 188 包）
//    2. PES_packet_length == 0 的不定长 PES（大帧，>64 KiB）
//    3. 从流中间接入（丢掉开头的 PAT/PMT，必须靠下一个 PSI 周期恢复）
//
//  素材（缺失时对应小节自动跳过）：
//    /tmp/test_h264_aud.h264   带 AUD 的 H.264 裸流
//    /tmp/test_nob.h265        H.265 裸流
//    /tmp/test.aac             ADTS 封装的 AAC
//    /tmp/ff_av.ts             ffmpeg 打的 TS，用来验证"能读别人的流"：
//        ffmpeg -y -i /tmp/test_h264_aud.h264 -i /tmp/test.aac \
//               -c copy -map 0:v -map 1:a /tmp/ff_av.ts
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <random>
#include <vector>

using namespace mpegmech;

namespace {

int g_fail = 0;
#define EXPECT(c, m) do { if (!(c)) { std::printf("  [FAIL] %s (line %d)\n", (m), __LINE__); ++g_fail; } } while (0)

std::vector<std::uint8_t> read_file(const char* p)
{
    std::vector<std::uint8_t> out;
    std::FILE* f = std::fopen(p, "rb");
    if (!f) { return out; }
    std::uint8_t b[65536];
    std::size_t  n;
    while ((n = std::fread(b, 1, sizeof(b), f)) > 0) {
        out.insert(out.end(), b, b + n);
    }
    std::fclose(f);
    return out;
}

/// 一帧 ES（录制下来的，用于比对）
struct Rec {
    int                       stream_id = 0;
    std::int64_t              pts       = 0;
    std::vector<std::uint8_t> data;
};

/// 把一个 ES 文件打成 TS，返回 TS 字节 + 每帧原始内容。
struct MuxResult {
    std::vector<std::uint8_t> ts;
    std::vector<Rec>          src;      ///< 送进复用器的帧（按顺序）
};

MuxResult mux_to_ts(const std::vector<std::uint8_t>& video,
                    Codec vcodec,
                    const std::vector<std::uint8_t>& audio = {})
{
    MuxResult r;
    std::error_code ec;

    MuxerConfig mc;
    mc.container = Container::ts;
    mc.psi_period_ms = 100;    // 测试里多打几次表，方便"中间接入"用例
    auto mx = Muxer::create(mc, [&r](span<const std::uint8_t> d) {
        r.ts.insert(r.ts.end(), d.begin(), d.end());
    }, ec);
    if (!mx) { return r; }

    // ---- 视频 ----
    EsSplitter::Config sc;
    sc.codec = vcodec;
    auto sp = EsSplitter::create(sc, ec);
    if (!sp) { return r; }

    StreamConfig vs;
    vs.codec = vcodec;
    auto vsid = mx->addStream(vs, ec);
    if (!vsid) { return r; }

    std::int64_t pts = 0;
    std::size_t  n   = 0;
    auto on_v = [&](span<const std::uint8_t> fr) {
        r.src.push_back({*vsid, pts, std::vector<std::uint8_t>(fr.begin(), fr.end())});
        mx->write(*vsid, n == 0 ? FrameFlag::key : FrameFlag::none, pts, pts, fr);
        pts += 3600;
        ++n;
    };
    sp->feed(span<const std::uint8_t>(video.data(), video.size()), on_v);
    sp->flush(on_v);

    // ---- 音频（裸 AAC，库加 ADTS）----
    if (!audio.empty()) {
        StreamConfig as;
        as.codec    = Codec::aac;
        as.add_adts = true;
        {
            // AudioSpecificConfig：AAC-LC / 44.1 kHz / 立体声
            static const std::uint8_t asc[] = {0x12, 0x10};
            as.extradata = span<const std::uint8_t>(asc, sizeof(asc));
        }
        auto asid = mx->addStream(as, ec);
        if (asid) {
            // 裸 AAC：按 ADTS 帧长切（这个素材每帧 1024 样本，直接按 ADTS 同步字切）
            std::size_t i = 0;
            std::int64_t apts = 0;
            while (i + 7 <= audio.size()) {
                const std::size_t flen =
                    ((static_cast<std::size_t>(audio[i + 3] & 0x03)) << 11) |
                    (static_cast<std::size_t>(audio[i + 4]) << 3) |
                    ((audio[i + 5] >> 5) & 0x07);
                if (flen < 7 || i + flen > audio.size()) { break; }
                // 剥掉 ADTS 头，送裸 ES 进去（add_adts=true 时库自己加）
                const std::size_t hdr = (audio[i + 1] & 0x01) ? 7 : 9;
                span<const std::uint8_t> es(&audio[i] + hdr, flen - hdr);
                r.src.push_back({*asid, apts, std::vector<std::uint8_t>(es.begin(), es.end())});
                mx->write(*asid, FrameFlag::key, apts, apts, es);
                apts += 1024 * 90000 / 44100;
                i += flen;
            }
        }
    }

    mx->flush();
    return r;
}

/// 按随机块长喂给解封装器，收集所有帧。
std::vector<Rec> demux_ts(const std::vector<std::uint8_t>& ts, unsigned seed,
                          std::uint64_t* skipped = nullptr,
                          Container*    sniffed = nullptr,
                          std::uint64_t* resyncs = nullptr)
{
    std::vector<Rec> got;
    std::error_code  ec;

    DemuxerConfig dc;
    dc.container = Container::unknown;      // 强制走嗅探
    auto dx = Demuxer::create(dc,
        [&got](const FrameInfo& fi, span<const std::uint8_t> es) {
            got.push_back({fi.stream_id, fi.pts,
                           std::vector<std::uint8_t>(es.begin(), es.end())});
        },
        nullptr, ec);
    if (!dx) { return got; }

    std::mt19937 rng(seed);
    std::size_t  i = 0;
    while (i < ts.size()) {
        std::size_t n = 1 + rng() % 400;                 // 1..400 字节，故意不对齐
        if (n > ts.size() - i) { n = ts.size() - i; }
        dx->feed(span<const std::uint8_t>(ts.data() + i, n));
        i += n;
    }
    dx->flush();

    if (skipped) { *skipped = dx->stats().bytes_skipped; }
    if (sniffed) { *sniffed = dx->container(); }
    if (resyncs) { *resyncs = dx->stats().resyncs; }
    return got;
}

/// 按 stream_id 分组（保持首次出现顺序）。
std::vector<std::vector<Rec>> group_by_stream(const std::vector<Rec>& v)
{
    std::vector<int>              ids;
    std::vector<std::vector<Rec>> g;
    for (const auto& r : v) {
        std::size_t k = 0;
        for (; k < ids.size(); ++k) {
            if (ids[k] == r.stream_id) { break; }
        }
        if (k == ids.size()) {
            ids.push_back(r.stream_id);
            g.emplace_back();
        }
        g[k].push_back(r);
    }
    return g;
}

/// 剥掉每帧前面的 ADTS 头（7 或 9 字节）。
std::vector<Rec> strip_adts(const std::vector<Rec>& v)
{
    std::vector<Rec> o;
    o.reserve(v.size());
    for (const auto& r : v) {
        std::size_t hdr = 0;
        if (r.data.size() >= 7 && r.data[0] == 0xFF && (r.data[1] & 0xF0) == 0xF0) {
            hdr = (r.data[1] & 0x01) ? 7u : 9u;
        }
        Rec n = r;
        if (hdr != 0 && r.data.size() >= hdr) {
            n.data.assign(r.data.begin() + static_cast<std::ptrdiff_t>(hdr), r.data.end());
        }
        o.push_back(std::move(n));
    }
    return o;
}

/// 逐字段比对
void compare(const char* name, const std::vector<Rec>& src, const std::vector<Rec>& got)
{
    std::printf("  %s: 送 %zu 帧 / 收 %zu 帧\n", name, src.size(), got.size());
    EXPECT(src.size() == got.size(), "帧数一致");
    if (src.size() != got.size()) { return; }

    std::size_t bad_bytes = 0, bad_pts = 0, first_bad = SIZE_MAX;
    for (std::size_t i = 0; i < src.size(); ++i) {
        if (src[i].data != got[i].data) {
            ++bad_bytes;
            if (first_bad == SIZE_MAX) { first_bad = i; }
        }
        if (src[i].pts != got[i].pts) { ++bad_pts; }
    }
    if (bad_bytes != 0) {
        std::printf("    第一个不一致的帧: #%zu（送 %zu 字节 / 收 %zu 字节）\n",
                    first_bad, src[first_bad].data.size(), got[first_bad].data.size());
    }
    EXPECT(bad_bytes == 0, "ES 逐字节一致");
    EXPECT(bad_pts == 0, "PTS 一致");
}

} // namespace

int main()
{
    std::printf("=== TS 往返 ===\n");

    // ---- 1. H264 ----
    {
        const auto raw = read_file("/tmp/test_h264_aud.h264");
        if (raw.empty()) {
            std::printf("跳过 H264（缺 /tmp/test_h264_aud.h264）\n");
        } else {
            const auto m = mux_to_ts(raw, Codec::h264);
            std::printf("H264 → TS: %zu 字节（%zu 包）\n", m.ts.size(), m.ts.size() / 188);
            EXPECT(m.ts.size() % 188 == 0, "TS 长度是 188 的整数倍");

            std::uint64_t skipped = 0, resyncs = 0;
            Container     sniffed = Container::unknown;
            const auto got = demux_ts(m.ts, 12345, &skipped, &sniffed, &resyncs);
            EXPECT(sniffed == Container::ts, "嗅探出 TS");
            EXPECT(resyncs == 0, "无误同步");
            compare("H264", m.src, got);
        }
    }

    // ---- 2. H265 ----
    {
        const auto raw = read_file("/tmp/test_nob.h265");
        if (raw.empty()) {
            std::printf("跳过 H265（缺 /tmp/test_nob.h265）\n");
        } else {
            const auto m = mux_to_ts(raw, Codec::h265);
            std::printf("H265 → TS: %zu 字节（%zu 包）\n", m.ts.size(), m.ts.size() / 188);
            const auto got = demux_ts(m.ts, 777);
            compare("H265", m.src, got);
        }
    }

    // ---- 3. 音视频混合 ----
    {
        const auto v = read_file("/tmp/test_h264_aud.h264");
        const auto a = read_file("/tmp/test.aac");
        if (v.empty() || a.empty()) {
            std::printf("跳过 AV（缺素材）\n");
        } else {
            const auto m = mux_to_ts(v, Codec::h264, a);
            std::printf("AV → TS: %zu 字节（%zu 包）\n", m.ts.size(), m.ts.size() / 188);
            if (std::FILE* f = std::fopen("/tmp/out_av.ts", "wb")) {
                std::fwrite(m.ts.data(), 1, m.ts.size(), f);
                std::fclose(f);
                std::printf("  已写 /tmp/out_av.ts（供 ffmpeg 交叉验证）\n");
            }

            // 音视频交错到达，顺序不可能与送进去的一致，所以按 stream 分组比。
            // 分组用 stream_id；再靠"首帧内容"把解出的组对应回源组（不依赖 id 大小）。
            const auto got = demux_ts(m.ts, 2024);

            const auto sg = group_by_stream(m.src);
            const auto gg = group_by_stream(got);
            std::printf("  源 %zu 组 / 解出 %zu 组\n", sg.size(), gg.size());
            EXPECT(sg.size() == 2, "一视频一音频");
            EXPECT(gg.size() == 2, "解出两条流");

            // 视频组：首帧内容能对上（TS 里视频 ES 是原样透传的）
            const std::vector<Rec>* gv = nullptr;
            const std::vector<Rec>* ga = nullptr;
            for (const auto& g : gg) {
                if (!g.empty() && !sg[0].empty() && g.front().data == sg[0].front().data) {
                    gv = &g;
                } else {
                    ga = &g;
                }
            }
            EXPECT(gv != nullptr, "找到视频组");
            EXPECT(ga != nullptr, "找到音频组");
            if (gv && ga) {
                compare("AV-视频", sg[0], *gv);

                // 音频：复用器 add_adts=true 会给裸 AAC 补 7 字节 ADTS 头，
                // 所以解出来的 payload 比送进去的多一个头 —— 比对前剥掉。
                // 顺带校验这层头是否合法（syncword 必须 0xFFF）。
                const auto stripped = strip_adts(*ga);
                std::size_t bad_sync = 0;
                for (const auto& r : *ga) {
                    if (r.data.size() < 7 || r.data[0] != 0xFF ||
                        (r.data[1] & 0xF0) != 0xF0) {
                        ++bad_sync;
                    }
                }
                std::printf("  ADTS 头非法: %zu / %zu\n", bad_sync, ga->size());
                EXPECT(bad_sync == 0, "音频每帧都带合法 ADTS 头");
                compare("AV-音频", sg[1], stripped);
            }
        }
    }

    // ---- 4. 从流中间接入 ----
    {
        const auto raw = read_file("/tmp/test_h264_aud.h264");
        if (!raw.empty()) {
            auto m = mux_to_ts(raw, Codec::h264);
            // 丢掉前 1200 字节（超过 6 个 TS 包），模拟从中间开始收
            std::vector<std::uint8_t> cut(m.ts.begin() + 1200, m.ts.end());
            const auto got = demux_ts(cut, 99);
            std::printf("中间接入：收到 %zu 帧（完整流 %zu 帧）\n", got.size(), m.src.size());
            EXPECT(!got.empty(), "中间接入能解出帧");
            EXPECT(got.size() < m.src.size(), "开头确实丢了一些");
            // 解出来的帧必须是**完整帧**：能在原始帧列表里找到
            std::size_t matched = 0;
            for (const auto& g : got) {
                for (const auto& s : m.src) {
                    if (s.data == g.data) { ++matched; break; }
                }
            }
            std::printf("  其中 %zu/%zu 帧与原始帧逐字节相同\n", matched, got.size());
            EXPECT(matched == got.size(), "中间接入解出的都是完整帧");
        }
    }

    // ---- 5. 第三方 TS（ffmpeg 生成）----
    //  自己封自己解容易"自洽地错"。这一段用的是 ffmpeg 打的 TS：PID 分配、
    //  PMT 描述符、PCR 位置都跟我们不一样，能过才算真的能读别人的流。
    {
        const auto ff = read_file("/tmp/ff_av.ts");
        if (ff.empty()) {
            std::printf("跳过第三方 TS（缺 /tmp/ff_av.ts）\n");
        } else {
            std::vector<Rec> got;
            std::uint64_t    resyncs = 0, errors = 0;
            std::error_code  ec;

            DemuxerConfig dc;
            dc.container = Container::unknown;
            auto dx = Demuxer::create(dc,
                [&got](const FrameInfo& fi, span<const std::uint8_t> es) {
                    got.push_back({fi.stream_id, fi.pts,
                                   std::vector<std::uint8_t>(es.begin(), es.end())});
                }, nullptr, ec);
            EXPECT(dx != nullptr, "创建解封装器");

            // 分段喂，块长故意不是 188 的倍数
            std::size_t i = 0;
            std::size_t n = 1;
            while (i < ff.size()) {
                std::size_t take = n;
                if (take > ff.size() - i) { take = ff.size() - i; }
                dx->feed(span<const std::uint8_t>(ff.data() + i, take));
                i += take;
                n = (n * 7 + 13) % 500 + 1;
            }
            dx->flush();
            resyncs = dx->stats().resyncs;
            errors  = dx->stats().parse_errors;

            const auto g = group_by_stream(got);
            std::printf("ffmpeg TS: %zu 字节 → %zu 条流 / %zu 帧（resync=%llu, err=%llu）\n",
                        ff.size(), g.size(), got.size(),
                        (unsigned long long)resyncs, (unsigned long long)errors);
            for (std::size_t k = 0; k < g.size(); ++k) {
                std::printf("  流 %d: %zu 帧\n", g[k].front().stream_id, g[k].size());
            }
            EXPECT(g.size() == 2, "认出两条流");
            EXPECT(resyncs == 0, "第三方流无误同步");
            EXPECT(errors == 0, "第三方流无结构错误");
            EXPECT(dx->container() == Container::ts, "嗅探出 TS");

            // 视频帧数必须和源文件一致（ffmpeg 是 -c copy，没重新编码）
            EXPECT(g.size() == 2 && g[0].size() == 75, "视频帧数与源一致（75）");

            // 音频：ffmpeg 把几十个 ADTS 帧攒成一条 PES，解出来必须是**单帧**，
            // 而不是"一坨 ADTS"。判据：首帧头合法，且 frame_length == 整个 payload。
            std::size_t multi = 0, badhdr = 0;
            const std::vector<Rec>* ag = (g.size() == 2) ? &g[1] : nullptr;
            if (ag != nullptr) {
                for (const auto& r : *ag) {
                    if (r.data.size() < 7 || r.data[0] != 0xFF ||
                        (r.data[1] & 0xF0) != 0xF0) {
                        ++badhdr;
                        continue;
                    }
                    const std::size_t flen =
                        (static_cast<std::size_t>(r.data[3] & 0x03) << 11) |
                        (static_cast<std::size_t>(r.data[4]) << 3) |
                        ((r.data[5] >> 5) & 0x07);
                    if (flen != r.data.size()) { ++multi; }
                }
                std::printf("  音频 %zu 帧：头非法 %zu / 多帧粘连 %zu\n",
                            ag->size(), badhdr, multi);
                EXPECT(badhdr == 0, "音频帧头合法");
                EXPECT(multi == 0, "每条 PES 里的 ADTS 已拆成单帧");
                EXPECT(ag->size() == 88, "音频帧数与源一致（88）");
            }
        }
    }

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
