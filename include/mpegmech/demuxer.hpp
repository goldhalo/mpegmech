// =============================================================================
//  mpegmech/demuxer.hpp —— Demuxer：容器流 -> ES 帧
// =============================================================================
//  文件    : mpegmech/demuxer.hpp
//  用途    : 把 PS / TS 字节流拆成 ES 帧，**接受任意切分的输入**。
//  编码    : UTF-8
//
//  ── 核心设计：为什么"非整帧输入"要做成库的责任 ──────────────────────────────
//  网络侧 `recv()` 回来的数据长度由内核决定，既不是 188 的整数倍，也不保证
//  落在 PES/包边界上。常见做法（libmpeg 的 ps_demuxer_input / ts_demuxer_input）
//  让调用方自己保存"消费不掉的尾巴"，再拼下一次；调用方一不留神就会：
//      * 漏存尾巴            -> 流错位，且**没有任何报错**
//      * 存了但拼错顺序      -> 同上
//      * TS 侧直接用 recv 块 -> 起点不是 0x47 时按 188 硬切，PID 全乱
//  本库把这三件事收到内部：
//      feed() 承诺**全部消费**（返回值只报错，不报"消费了多少"），
//      内部维护环形暂存 + 同步状态机，负责找边界、拼包、恢复失步。
//
//  ── 同步策略 ────────────────────────────────────────────────────────────────
//  TS：找 k 使 data[k] == 0x47 且 data[k+188] == 0x47 且 data[k+376] == 0x47
//      （连续包数由 `ts_sync_packets` 配置，默认 3）。命中后以 k 为包边界，
//      [0, k) 计入 bytes_skipped。运行期若某个包的 sync_byte 不是 0x47，
//      立即回到搜索态（计入 resyncs），而不是继续按 188 硬切。
//  PS：找 `00 00 01 BA`。MPEG-2 的 pack_header 在每个 pack 起始，MPEG-1 兼容
//      流同样有；命中前的内容计入 bytes_skipped。
//
//  ── 回调契约 ────────────────────────────────────────────────────────────────
//  * `frame_cb` / `stream_cb` 在 `feed()` / `flush()` 的调用栈内**同步执行**，
//    串行、不并发；
//  * 回调里**不得**调用本对象的 feed / flush / reset（重入会破坏内部状态）；
//  * `payload` 指向库内部缓冲，**回调返回后即失效**。需要留存请自行拷贝；
//  * 回调抛异常会穿过 feed() 传播 —— 库内部不捕获（与 netmech 一致）。
//
//  ★★ 回调对象的生命周期（最容易踩的一条）★★
//  ---------------------------------------------------------------------------
//  构造时传入的 `std::function` 会被**拷贝并持有**到本对象销毁。所以：
//
//      lambda 捕获的东西（尤其 `[&]` 捕获的局部变量）必须活到 Demuxer 销毁。
//
//  反例（**会崩**）：
//
//      void setup() {
//          int counter = 0;
//          auto d = Demuxer::create(cfg, [&counter](auto f, ...) {
//              counter++;                       // ← setup() 返回后 counter 已死
//          }, ec);
//      }   // setup 返回，d 还活着，下次回调就访问野指针
//
//  正确做法（三选一）：
//    1. 捕获 `shared_ptr`（或 `weak_ptr` + 判空）
//    2. 捕获值（小对象，`[=]`），或把状态放到堆上
//    3. 把回调对象定义在 Demuxer 的生命周期之外
//
//  中途要解绑（比如 owner 先死了）：调 `setHandler(nullptr)`。
//
//  ★★ 析构不回调 ★★
//  ---------------------------------------------------------------------------
//  `~Demuxer()` **不触发任何回调**（包括不 flush 残留帧）。
//
//  这是刻意的：析构期间触发回调，调用方可能在回调里访问正在析构的对象
//  （引用计数、成员变量），必崩。我们上一版就是这么崩的。
//
//  需要交出最后一帧 → **显式调 `flush()`**，再销毁。
//
//  ★★ 致命错误后的状态 ★★
//  ---------------------------------------------------------------------------
//  遇到 L4 错误（out_of_memory / internal_error）后，对象进入 **failed 状态**：
//  后续 feed/flush 立刻返回同一个错误码。
//  **调 `reset()` 才能恢复**（清空缓冲和状态机，回调保留）。
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <system_error>

#include "mpegmech/error.hpp"
#include "mpegmech/span.hpp"
#include "mpegmech/types.hpp"

namespace mpegmech {

// -----------------------------------------------------------------------------
// 配置
// -----------------------------------------------------------------------------

struct DemuxerConfig {
    /// 复用格式。`unknown` = 按首包内容自动嗅探（推荐）。
    /// 明确知道格式时直接指定，可省掉一次嗅探与少量延迟。
    Container container = Container::unknown;

    /// 内部暂存上限（字节）。超过后按"丢最旧"策略腾空间并计 buffer_overflow，
    /// 同时回到同步搜索态。默认 4 MiB（约 1 秒的 32 Mbps 码流）。
    ///
    /// @note 触发上限通常意味着调用方喂得太快或下游消费太慢，属于异常，
    ///       但**不终止流** —— 恢复同步后继续工作。
    std::size_t max_buffered_bytes = 4u * 1024u * 1024u;

    /// 单帧上限（字节）。超长视为异常帧：截断后以 incomplete 标志吐出，计 parse_errors。
    /// 默认 4 MiB（1080p IDR 通常 < 1 MiB）。
    std::size_t max_frame_bytes = 4u * 1024u * 1024u;

    /// TS 判定同步所需的连续包数（>= 2）。默认 3。
    /// 提高可降低误同步概率，代价是启动时多等 (n-1)*188 字节。
    std::size_t ts_sync_packets = 3;

    /// 连续失步达到该字节数即认为本流已无法恢复，feed() 返回 sync_lost。
    /// 默认 256 KiB。设为 0 表示永不放弃（一直搜索）。
    std::size_t ts_max_resync_bytes = 256u * 1024u;

    /// `flush()` 时是否把未结束的 PES 也吐出来（带 incomplete 标志）。
    /// 默认 false —— 截断的帧对下游没有价值，且可能被误当完整帧解码。
    bool emit_partial_on_flush = false;

    /// 是否在 `aud` 边界切分视频帧（TS/PS 里一帧可能含多个 AUD）。
    /// 默认 false：按 PES 边界切，与容器保持一致（推荐）。
    bool split_video_by_aud = false;
};

// -----------------------------------------------------------------------------
// 统计
// -----------------------------------------------------------------------------

/// 解封装统计快照（只增不减，`reset()` 时清零）。
struct DemuxStats {
    std::uint64_t bytes_in          = 0;  ///< feed() 收到的总字节
    std::uint64_t bytes_skipped     = 0;  ///< 同步搜索期间丢弃的字节
    std::uint64_t frames_out        = 0;  ///< 回调出的帧数
    std::uint64_t resyncs           = 0;  ///< 运行期失步次数
    std::uint64_t parse_errors      = 0;  ///< 结构非法次数（已跳过并继续）
    std::uint64_t overflow_drops    = 0;  ///< 因 max_buffered_bytes 触发的丢弃次数
    std::uint64_t incomplete_frames = 0;  ///< 带 incomplete 标志的帧数
};

// -----------------------------------------------------------------------------
// Demuxer
// -----------------------------------------------------------------------------

/// PS / TS 解封装器。
///
/// 典型用法（网络接收，块长度任意）：
/// ```cpp
/// DemuxerConfig cfg;                       // 默认自动嗅探
/// std::error_code ec;
/// auto dx = Demuxer::create(cfg,
///     [](const FrameInfo& fi, span<const std::uint8_t> es) {
///         // fi.codec / fi.pts / fi.flags 已填好；es 只在本次回调内有效
///         onFrame(fi, es);
///     },
///     [](const StreamInfo& si) { /* 新流出现或 codec 变化 */ },
///     ec);
///
/// while (auto n = sock.recv(buf, sizeof(buf)))   // 不需要对齐！
///     dx->feed(span<const std::uint8_t>(buf, n));
/// dx->flush();                                   // 流结束
/// ```
///
/// @note 本类不可拷贝。`create` 失败时返回 nullptr，原因见 `ec`。
class Demuxer {
public:
    /// 帧回调。`payload` 是 **AnnexB / ADTS / 裸 ES**（与容器内形态一致，
    /// 不做转封装），生命周期见文件头说明。
    using FrameCallback  = std::function<void(const FrameInfo&, span<const std::uint8_t>)>;

    /// 流回调。发现新流、或已有流的 codec/extradata 变化时触发。
    /// 保证在第一条该流的帧回调**之前**触发。
    using StreamCallback = std::function<void(const StreamInfo&)>;

    /// 创建解封装器。
    ///
    /// @param cfg       配置
    /// @param on_frame  帧回调（必填）
    /// @param on_stream 流回调（可空）
    /// @param ec        失败原因
    /// @return 成功返回对象，失败返回 nullptr 且 `ec` 非 0
    [[nodiscard]] static std::unique_ptr<Demuxer> create(const DemuxerConfig& cfg,
                                                         FrameCallback on_frame,
                                                         StreamCallback on_stream,
                                                         std::error_code& ec);

    ~Demuxer();

    Demuxer(Demuxer&&) noexcept;
    Demuxer& operator=(Demuxer&&) noexcept;
    Demuxer(const Demuxer&)            = delete;
    Demuxer& operator=(const Demuxer&) = delete;

    /// 喂入任意长度的数据。**全部消费**（内部暂存未成帧的尾巴）。
    ///
    /// @return 0 表示正常（包含"数据不足，等待更多"）；
    ///         sync_lost 表示连续失步超限，本流已放弃（需 reset() 才能复用）；
    ///         其余为输入/配置错误。
    ///
    /// @note 帧/流回调在本调用内同步触发。
    std::error_code feed(span<const std::uint8_t> data);

    /// 通知输入结束：把内部还能凑成完整单元的部分处理掉。
    /// `emit_partial_on_flush=true` 时残余 PES 也会以 incomplete 吐出。
    std::error_code flush();

    /// 已识别的容器格式（嗅探完成前为 `unknown`）。
    [[nodiscard]] Container container() const noexcept;

    /// 当前内部暂存字节数（调试/水位观测用）。
    [[nodiscard]] std::size_t buffered() const noexcept;

    /// 更换回调。传空（`nullptr`）则**停止回调**（feed 仍然消费数据）。
    ///
    /// @par 什么时候用
    ///  owner 先于 Demuxer 销毁时（比如连接断开但对象池还要回收 Demuxer）。
    ///  调一次 `setHandler(nullptr)` 就彻底切断了回调对 owner 的依赖。
    ///
    /// @note 不能在回调里调用（会破坏 `std::function` 自身的调用栈）。
    void setHandler(FrameCallback on_frame);

    /// 复位为**刚 create 完的状态**。
    ///
    /// 清空：缓冲、同步状态机、PSI 表（PAT/PMT）、已识别的流列表、**统计**。
    /// 保留：配置（cfg）、回调（on_frame）。
    ///
    /// @par 什么时候用
    ///  - L4 致命错误后恢复 —— 失败时内部状态可能半截，清掉比保留安全
    ///  - 换一路新流 —— 复用对象，避免反复 create/destroy
    ///
    /// @note 统计**清零**。原因：复位后的数据属于"新的开始"，把上一路流的
    ///       计数混进来只会误导。要跨复位观察问题，调用方自己累加即可 ——
    ///       而真正该看的其实是 `stats()` 里的 `sync_lost` / `buffer_overflow`
    ///       / `malformed_packets`，那些才反映"流本身有什么问题"。
    void reset() noexcept;

    /// 是否处于 failed 状态（L4 之后、reset 之前）。
    [[nodiscard]] bool failed() const noexcept;

    /// 统计快照。
    [[nodiscard]] const DemuxStats& stats() const noexcept;

private:
    struct Impl;
    explicit Demuxer(std::unique_ptr<Impl> impl) noexcept;

    std::unique_ptr<Impl> impl_;
};

} // namespace mpegmech
