// =============================================================================
//  soak.cpp —— 长时间稳定性：反复复用 / 反复重建，观察内存与吞吐
// =============================================================================
//  库层面的泄漏不像服务那样容易暴露：跑一遍是对的，跑十万遍可能慢慢涨。
//  这个程序只做一件事 —— 用同一份数据无限循环，每 10 秒打一行 RSS。
//
//  两种负载分开测，因为它们的泄漏点不同：
//
//    reuse  一个 Demuxer 长期存活，喂完一轮 flush + reset 再来
//           → 暴露"缓冲/状态机/流表"没清干净的累计增长
//    churn  每轮 create → feed → destroy
//           → 暴露构造析构路径上的泄漏（比如回调持有、map 没释放）
//
//  用法：
//      mpegmech_soak [--seconds N] [--mode reuse|churn|both] [--data DIR]
//
//  判读：看 RSS 曲线的**形状**，不是看斜率。
//    启动期爬升然后走平 = 正常（分配器热身、容量稳定在高水位）
//    一直线性上涨      = 泄漏
//
//  注意：程序是单线程的，进度打印直接在主循环里做 —— 省掉一个监视线程
//  和随之而来的竞态（早期版本用线程读计数器，属于 data race）。
// =============================================================================
#include <mpegmech/demuxer.hpp>
#include <mpegmech/es_splitter.hpp>
#include <mpegmech/muxer.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#if defined(_WIN32)
#  include <windows.h>
#  include <psapi.h>
#  pragma comment(lib, "psapi.lib")
#endif

using namespace mpegmech;

namespace {

using Clock = std::chrono::steady_clock;

/// 当前进程常驻内存（字节）。
std::size_t rss_bytes()
{
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS pmc{};
    if (::GetProcessMemoryInfo(::GetCurrentProcess(), &pmc, sizeof(pmc))) {
        return pmc.WorkingSetSize;
    }
    return 0;
#else
    // 读 /proc/self/statm 第二列（resident pages），比遍历 vma 便宜得多。
    // 页大小这里按 4096 算 —— 目标平台 x86_64 / aarch64 都是 4 KiB。
    std::FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) { return 0; }
    unsigned long total = 0, resident = 0;
    const int     n = std::fscanf(f, "%lu %lu", &total, &resident);
    std::fclose(f);
    if (n != 2) { return 0; }
    return static_cast<std::size_t>(resident) * 4096u;
#endif
}

std::string g_data_dir = ".";

std::vector<std::uint8_t> read_file(const char* name)
{
    std::vector<std::string> dirs;
    if (const char* d = std::getenv("MPEGMECH_TEST_DATA")) { dirs.emplace_back(d); }
    dirs.emplace_back(g_data_dir);
    dirs.emplace_back("/tmp");
    dirs.emplace_back(".");

    for (const auto& d : dirs) {
        const std::string p = d + "/" + name;
        std::FILE* f = std::fopen(p.c_str(), "rb");
        if (!f) { continue; }
        g_data_dir = d;
        std::vector<std::uint8_t> out;
        std::uint8_t b[65536];
        std::size_t  n;
        while ((n = std::fread(b, 1, sizeof(b), f)) > 0) { out.insert(out.end(), b, b + n); }
        std::fclose(f);
        return out;
    }
    return {};
}

/// 把裸流打成 TS（只做一次，之后循环复用这份字节）。
std::vector<std::uint8_t> build_ts(const std::vector<std::uint8_t>& raw, Codec codec)
{
    std::vector<std::uint8_t> ts;
    std::error_code           ec;

    MuxerConfig mc;
    mc.container = Container::ts;
    auto mx = Muxer::create(mc, [&ts](span<const std::uint8_t> d) {
        ts.insert(ts.end(), d.begin(), d.end());
    }, ec);
    if (!mx) { return ts; }

    EsSplitter::Config sc;
    sc.codec = codec;
    auto sp = EsSplitter::create(sc, ec);
    if (!sp) { return ts; }

    StreamConfig st;
    st.codec = codec;
    auto sid = mx->addStream(st, ec);
    if (!sid) { return ts; }

    std::int64_t pts = 0;
    std::size_t  n   = 0;
    auto on_f = [&](span<const std::uint8_t> fr) {
        mx->write(*sid, n == 0 ? FrameFlag::key : FrameFlag::none, pts, pts, fr);
        pts += 3600;
        ++n;
    };
    sp->feed(span<const std::uint8_t>(raw.data(), raw.size()), on_f);
    sp->flush(on_f);
    mx->flush();
    return ts;
}

struct Result {
    std::uint64_t rounds = 0;
    std::uint64_t frames = 0;
    double        secs   = 0;
};

/// 一个长期存活的 Demuxer，反复喂 + reset。
Result run_reuse(const std::vector<std::uint8_t>& ts, int seconds)
{
    Result          r;
    DemuxerConfig   dc;
    std::error_code ec;

    std::uint64_t frames = 0;
    auto dx = Demuxer::create(dc,
        [&frames](const FrameInfo&, span<const std::uint8_t>) { ++frames; },
        nullptr, ec);
    if (!dx) { return r; }

    std::mt19937 rng(1234);
    const auto   t0  = Clock::now();
    auto         nxt = t0 + std::chrono::seconds(10);

    for (;;) {
        std::size_t i = 0;
        while (i < ts.size()) {
            std::size_t n = 1 + rng() % 1400;
            if (n > ts.size() - i) { n = ts.size() - i; }
            dx->feed(span<const std::uint8_t>(ts.data() + i, n));
            i += n;
        }
        dx->flush();
        dx->reset();                      // 模拟"换一路流"
        ++r.rounds;

        const auto now = Clock::now();
        if (now > nxt) {
            std::printf("  t=%3llds  rss=%6.1f MB  rounds=%llu  frames=%llu\n",
                        static_cast<long long>(
                            std::chrono::duration_cast<std::chrono::seconds>(now - t0).count()),
                        static_cast<double>(rss_bytes()) / 1048576.0,
                        static_cast<unsigned long long>(r.rounds),
                        static_cast<unsigned long long>(frames));
            std::fflush(stdout);
            nxt = now + std::chrono::seconds(10);
            if (now - t0 > std::chrono::seconds(seconds)) { break; }
        }
    }

    r.frames = frames;
    r.secs   = std::chrono::duration<double>(Clock::now() - t0).count();
    return r;
}

/// 每轮重新 create / destroy。
Result run_churn(const std::vector<std::uint8_t>& ts, int seconds)
{
    Result       r;
    std::mt19937 rng(4321);
    const auto   t0  = Clock::now();
    auto         nxt = t0 + std::chrono::seconds(10);

    for (;;) {
        DemuxerConfig   dc;
        std::error_code ec;
        std::uint64_t   frames = 0;

        auto dx = Demuxer::create(dc,
            [&frames](const FrameInfo&, span<const std::uint8_t>) { ++frames; },
            nullptr, ec);
        if (!dx) { break; }

        std::size_t i = 0;
        while (i < ts.size()) {
            std::size_t n = 1 + rng() % 1400;
            if (n > ts.size() - i) { n = ts.size() - i; }
            dx->feed(span<const std::uint8_t>(ts.data() + i, n));
            i += n;
        }
        dx->flush();
        r.frames += frames;
        ++r.rounds;

        const auto now = Clock::now();
        if (now > nxt) {
            std::printf("  t=%3llds  rss=%6.1f MB  rounds=%llu  frames=%llu\n",
                        static_cast<long long>(
                            std::chrono::duration_cast<std::chrono::seconds>(now - t0).count()),
                        static_cast<double>(rss_bytes()) / 1048576.0,
                        static_cast<unsigned long long>(r.rounds),
                        static_cast<unsigned long long>(r.frames));
            std::fflush(stdout);
            nxt = now + std::chrono::seconds(10);
            if (now - t0 > std::chrono::seconds(seconds)) { break; }
        }
    }

    r.secs = std::chrono::duration<double>(Clock::now() - t0).count();
    return r;
}

void report(const char* tag, const Result& r, std::size_t bytes_per_round)
{
    const double mb = static_cast<double>(bytes_per_round * r.rounds) / 1048576.0;
    std::printf("  -> %llu 轮 / %llu 帧 / %.1f MB 输入 / %.1f s / %.2f MB/s\n",
                static_cast<unsigned long long>(r.rounds),
                static_cast<unsigned long long>(r.frames),
                mb, r.secs, r.secs > 0 ? mb / r.secs : 0.0);
    std::printf("     %s 每轮 %.3f ms\n", tag,
                r.rounds > 0 ? r.secs * 1000.0 / static_cast<double>(r.rounds) : 0.0);
}

} // namespace

int main(int argc, char** argv)
{
    int         seconds = 600;
    std::string mode    = "both";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            seconds = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            mode = argv[++i];
        } else if (std::strcmp(argv[i], "--data") == 0 && i + 1 < argc) {
            g_data_dir = argv[++i];
        } else {
            std::printf("用法: %s [--seconds N] [--mode reuse|churn|both] [--data DIR]\n",
                        argv[0]);
            return 2;
        }
    }
    if (seconds <= 0) { seconds = 1; }

    const auto raw = read_file("test_h264_aud.h264");
    if (raw.empty()) {
        std::printf("缺素材 test_h264_aud.h264（用 --data 或 MPEGMECH_TEST_DATA 指定目录）\n");
        return 1;
    }

    const auto ts = build_ts(raw, Codec::h264);
    if (ts.empty()) {
        std::printf("打 TS 失败\n");
        return 1;
    }

    const std::size_t rss0 = rss_bytes();
    std::printf("素材 %zu 字节 → TS %zu 字节（%zu 个包）\n",
                raw.size(), ts.size(), ts.size() / 188);
    std::printf("起始 RSS %.1f MB，每 10 秒采样一次\n",
                static_cast<double>(rss0) / 1048576.0);

    if (mode == "reuse" || mode == "both") {
        std::printf("\n[reuse] 一个 Demuxer 反复喂 + reset\n");
        report("[reuse]", run_reuse(ts, seconds), ts.size());
    }
    if (mode == "churn" || mode == "both") {
        std::printf("\n[churn] 每轮 create / destroy\n");
        report("[churn]", run_churn(ts, seconds), ts.size());
    }

    const std::size_t rss1 = rss_bytes();
    std::printf("\nRSS：开始 %.1f MB → 结束 %.1f MB（%+.1f MB）\n",
                static_cast<double>(rss0) / 1048576.0,
                static_cast<double>(rss1) / 1048576.0,
                (static_cast<double>(rss1) - static_cast<double>(rss0)) / 1048576.0);
    std::printf("判读：看曲线是否走平。启动期爬升后持平正常，持续线性上涨才是泄漏。\n");
    return 0;
}
