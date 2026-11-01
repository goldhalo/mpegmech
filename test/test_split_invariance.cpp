// =============================================================================
//  test_split_invariance.cpp —— 切分不变性
// =============================================================================
//  本库的招牌承诺是"feed() 接受任意切分"，那就得证明 **切法不影响结果**。
//
//  做法：同一段容器流，用十几种块长喂进去（1 字节 / 188 边界 / 189 跨包 /
//  随机 …），把所有输出帧排成序列逐字段比对。任一处不一致就说明内部缓冲
//  或状态机在某个切分点上有洞 —— 这类 bug 在"整块喂"的测试里永远看不到。
//
//  同时值这个测试盯的两个具体点：
//    * 1 字节喂：每次 feed 都只推进一点点，最容易暴露"等够再解析"的边界错
//    * 188±1 喂：正好卡在 TS 包边界上/差一个字节，同步与 PES 累积的经典坑
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace mpegmech;

namespace {

int g_fail = 0;
#define EXPECT(c, m) do { if (!(c)) { std::printf("  [FAIL] %s (line %d)\n", (m), __LINE__); ++g_fail; } } while (0)

std::string g_data_dir = ".";

std::string find_fixture(const char* name)
{
    std::vector<std::string> dirs;
    if (const char* d = std::getenv("MPEGMECH_TEST_DATA")) { dirs.emplace_back(d); }
    dirs.emplace_back("/tmp");
    dirs.emplace_back(".");
    for (const auto& d : dirs) {
        const std::string p = d + "/" + name;
        if (std::FILE* f = std::fopen(p.c_str(), "rb")) {
            std::fclose(f);
            g_data_dir = d;
            return p;
        }
    }
    return {};
}

std::vector<std::uint8_t> read_file(const char* name)
{
    std::vector<std::uint8_t> out;
    const std::string         p = find_fixture(name);
    if (p.empty()) { return out; }
    std::FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) { return out; }
    std::uint8_t b[65536];
    std::size_t  n;
    while ((n = std::fread(b, 1, sizeof(b), f)) > 0) { out.insert(out.end(), b, b + n); }
    std::fclose(f);
    return out;
}

/// 一帧解析结果（用于跨切分比对）
struct Out {
    int                       sid = 0;
    std::int64_t              pts = 0;
    std::vector<std::uint8_t> data;
};

std::vector<std::uint8_t> make_container(const std::vector<std::uint8_t>& video,
                                         Codec vcodec, Container c,
                                         const std::vector<std::uint8_t>& audio = {})
{
    std::vector<std::uint8_t> out;
    std::error_code           ec;

    MuxerConfig mc;
    mc.container     = c;
    mc.psi_period_ms = 100;
    auto mx = Muxer::create(mc, [&out](span<const std::uint8_t> d) {
        out.insert(out.end(), d.begin(), d.end());
    }, ec);
    if (!mx) { return out; }

    EsSplitter::Config sc;
    sc.codec = vcodec;
    auto sp = EsSplitter::create(sc, ec);
    if (!sp) { return out; }

    StreamConfig vs;
    vs.codec = vcodec;
    auto vsid = mx->addStream(vs, ec);
    if (!vsid) { return out; }

    std::int64_t pts = 0;
    std::size_t  n   = 0;
    auto on_v = [&](span<const std::uint8_t> fr) {
        mx->write(*vsid, n == 0 ? FrameFlag::key : FrameFlag::none, pts, pts, fr);
        pts += 3600;
        ++n;
    };
    sp->feed(span<const std::uint8_t>(video.data(), video.size()), on_v);
    sp->flush(on_v);

    if (!audio.empty()) {
        StreamConfig as;
        as.codec    = Codec::aac;
        as.add_adts = true;
        static const std::uint8_t asc[] = {0x12, 0x10};
        as.extradata = span<const std::uint8_t>(asc, sizeof(asc));
        auto asid = mx->addStream(as, ec);
        if (asid) {
            std::size_t  i = 0;
            std::int64_t apts = 0;
            while (i + 7 <= audio.size()) {
                const std::size_t flen =
                    (static_cast<std::size_t>(audio[i + 3] & 0x03) << 11) |
                    (static_cast<std::size_t>(audio[i + 4]) << 3) |
                    ((audio[i + 5] >> 5) & 0x07);
                if (flen < 7 || i + flen > audio.size()) { break; }
                const std::size_t hdr = (audio[i + 1] & 0x01) ? 7 : 9;
                span<const std::uint8_t> es(&audio[i] + hdr, flen - hdr);
                mx->write(*asid, FrameFlag::key, apts, apts, es);
                apts += 1024 * 90000 / 44100;
                i += flen;
            }
        }
    }

    mx->flush();
    return out;
}

/// 用固定块长（fixed != 0）或随机块长（fixed == 0，用 seed）喂完整流。
std::vector<Out> demux_with(const std::vector<std::uint8_t>& s,
                            std::size_t fixed, unsigned seed)
{
    std::vector<Out> out;
    std::error_code  ec;

    DemuxerConfig dc;                      // container = unknown，走嗅探
    auto dx = Demuxer::create(dc,
        [&out](const FrameInfo& fi, span<const std::uint8_t> es) {
            Out o;
            o.sid  = fi.stream_id;
            o.pts  = fi.pts;
            o.data.assign(es.begin(), es.end());
            out.push_back(std::move(o));
        }, nullptr, ec);
    if (!dx) { return out; }

    std::mt19937 rng(seed);
    std::size_t  i = 0;
    while (i < s.size()) {
        std::size_t n = fixed != 0 ? fixed : (1 + rng() % 977);
        if (n > s.size() - i) { n = s.size() - i; }
        dx->feed(span<const std::uint8_t>(s.data() + i, n));
        i += n;
    }
    dx->flush();
    return out;
}

/// 比对两轮输出
bool same(const std::vector<Out>& a, const std::vector<Out>& b, int* first_diff)
{
    if (a.size() != b.size()) {
        *first_diff = static_cast<int>(a.size() < b.size() ? a.size() : b.size());
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].sid != b[i].sid || a[i].pts != b[i].pts || a[i].data != b[i].data) {
            *first_diff = static_cast<int>(i);
            return false;
        }
    }
    return true;
}

struct Pattern {
    const char* name;
    std::size_t fixed;    ///< 0 = 随机
    unsigned    seed;
};

void check_invariance(const char* what, const std::vector<std::uint8_t>& stream)
{
    static const Pattern pats[] = {
        {"1 B",        1,    0},
        {"2 B",        2,    0},
        {"3 B",        3,    0},
        {"7 B",        7,    0},
        {"183 B",    183,    0},
        {"184 B",    184,    0},
        {"187 B",    187,    0},
        {"188 B",    188,    0},   // TS 包长
        {"189 B",    189,    0},   // 每次都跨包
        {"376 B",    376,    0},   // 2 个包
        {"1000 B",  1000,    0},
        {"4096 B",  4096,    0},
        {"rand#1",     0,    1},
        {"rand#2",     0, 99991},
    };

    const std::size_t np = sizeof(pats) / sizeof(pats[0]);

    // 基准：用一个足够大的固定块（整块）
    const auto base = demux_with(stream, stream.size() ? stream.size() : 1, 0);
    std::printf("%s：%zu 字节 → 基准 %zu 帧\n", what, stream.size(), base.size());
    EXPECT(!base.empty(), "基准非空");

    for (std::size_t k = 0; k < np; ++k) {
        const auto r = demux_with(stream, pats[k].fixed, pats[k].seed);
        int        at = -1;
        const bool ok = same(base, r, &at);
        if (!ok) {
            std::printf("  %-8s 帧数 %zu（基准 %zu），首处不同 %d\n",
                        pats[k].name, r.size(), base.size(), at);
        }
        EXPECT(ok, pats[k].name);
    }
}

} // namespace

int main()
{
    std::printf("=== 切分不变性 ===\n");

    const auto v = read_file("test_h264_aud.h264");
    if (v.empty()) {
        std::printf("跳过（缺 test_h264_aud.h264，可用 MPEGMECH_TEST_DATA 指定目录）\n");
        return 0;
    }
    const auto a = read_file("test.aac");

    // ---- PS 视频 ----
    check_invariance("PS/H264", make_container(v, Codec::h264, Container::ps));

    // ---- TS 视频 ----
    check_invariance("TS/H264", make_container(v, Codec::h264, Container::ts));

    // ---- TS 音视频 ----
    if (!a.empty()) {
        check_invariance("TS/AV", make_container(v, Codec::h264, Container::ts, a));
    }

    // ---- TS 从中间接入（没有 PAT/PMT 开头，考验"等 PSI"期间的分片处理）----
    {
        auto ts = make_container(v, Codec::h264, Container::ts);
        if (ts.size() > 2000) {
            std::vector<std::uint8_t> cut(ts.begin() + 1300, ts.end());
            check_invariance("TS/中间接入", cut);
        }
    }

    std::printf("\n%s（%d 个失败）\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
