// =============================================================================
//  mpegmech/muxer.hpp —— Muxer：ES 帧 -> 容器流
// =============================================================================
//  文件    : mpegmech/muxer.hpp
//  用途    : 把 ES 帧打包成 PS / TS 字节流。
//  编码    : UTF-8
//
//  ── 设计说明 ────────────────────────────────────────────────────────────────
//  * **输入是"整帧"**：mux 需要知道帧边界才能填 PES 长度、PTS/DTS。调用方
//    必须是"一帧一次 write"。若手上只有字节流（例如从 RTP 收到的分片），
//    先用上层拆帧器（AnnexB 起始码扫描 / ADTS 长度字段）还原帧边界。
//    这与 demux 侧"任意切分"的能力不矛盾：demux 的输入是容器流（自带边界
//    信息），mux 的输入是裸 ES（不含长度字段），两者条件不同。
//  * **输出走回调**：每次回调交出一段**完整可发送**的字节（TS 是 188 的整数倍，
//    PS 是一个完整 pack）。调用方可以直接 send / write，不必再攒。
//  * 时间戳单位 90 kHz，调用方负责单调递增；本库不做时间基重映射，
//    只在 TS 侧按 `pcr_period_ms` 插入 PCR（取最近一个写入帧的 dts）。
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <system_error>

#include "mpegmech/error.hpp"
#include "mpegmech/span.hpp"
#include "mpegmech/types.hpp"

namespace mpegmech {

// -----------------------------------------------------------------------------
// 流配置
// -----------------------------------------------------------------------------

/// 一条待复用 ES 流的描述。
struct StreamConfig {
    /// 编码类型（必填，决定 stream_type / stream_id / 是否需要 AUD）。
    Codec codec = Codec::unknown;

    /// TS 的 PID。0 = 自动分配（视频从 0x0100 起，音频从 0x0110 起）。
    /// @note 手动指定时不得与 PAT(0x0000)/PMT(0x1000)/NULL(0x1FFF) 冲突。
    std::uint16_t pid = 0;

    /// PS 的 stream_id。0 = 自动（视频 0xE0 起，音频 0xC0 起）。
    std::uint8_t stream_id = 0;

    /// 额外数据（仅 **PS** 与部分 codec 需要）：
    ///   * H.264 / H.265：可给 VPS/SPS/PPS（AnnexB），用于生成 PS 的 PSM 描述符；
    ///   * AAC：AudioSpecificConfig（2~5 字节），用于生成 PSM 与 ADTS 头。
    /// TS 场景忽略（参数集应随码流走，PMT 描述符由实现按需生成）。
    ///
    /// @note `addStream` 内部**拷贝**，调用方随后可释放。
    span<const std::uint8_t> extradata{};

    /// AAC 是否需要本库补 ADTS 头。
    ///   true  = 调用方给裸 AAC（无 ADTS），库加 7 字节头后入 PES（PS 常用）；
    ///   false = 调用方给的已是 ADTS，库原样透传。
    /// TS 场景恒按 false（TS 里的 AAC 必须带 ADTS）。
    bool add_adts = false;
};

// -----------------------------------------------------------------------------
// 复用器配置
// -----------------------------------------------------------------------------

struct MuxerConfig {
    /// 输出容器。`unknown` 视为非法。
    Container container = Container::ts;

    /// TS：节目号（1..0xFFFF）。默认 1。
    std::uint16_t program_number = 1;

    /// TS：PMT 的 PID。默认 0x1000。
    std::uint16_t pmt_pid = 0x1000;

    /// TS：PCR 承载 PID。0 = 用第一条视频流（无视频则用第一条音频）。
    std::uint16_t pcr_pid = 0;

    /// TS：PCR 插入间隔（毫秒）。0 = 不插 PCR。默认 40 ms（约 25 帧/秒）。
    std::int32_t pcr_period_ms = 40;

    /// TS：PAT/PMT 重发间隔（毫秒）。默认 100 ms。设为 0 只在流启动时发一次。
    std::int32_t psi_period_ms = 100;

    /// PS：单个 pack 的目标上限（字节），超过即切下一个 pack。默认 64 KiB。
    std::size_t ps_pack_bytes = 64u * 1024u;

    /// PS：是否输出 system_header（0x000001BB）。默认 true（兼容性更好）。
    bool ps_system_header = true;

    /// 是否允许同一 stream 的 pts 回退（B 帧场景 dts < pts 属正常，但 pts 必须单调）。
    /// 默认 false —— 回退视为调用方错误，`write` 返回 malformed。
    bool allow_pts_rewind = false;
};

// -----------------------------------------------------------------------------
// 统计
// -----------------------------------------------------------------------------

/// 复用统计快照。
struct MuxStats {
    std::uint64_t frames_in    = 0;  ///< write() 成功入队的帧数
    std::uint64_t bytes_in     = 0;  ///< 入队 ES 的总字节
    std::uint64_t bytes_out    = 0;  ///< 回调交出的总字节
    std::uint64_t packets_out  = 0;  ///< TS: 188 字节包数；PS: pack 数
    std::uint64_t psi_sent     = 0;  ///< PAT/PMT 发送次数（PS: PSM 次数）
    std::uint64_t pcr_sent     = 0;  ///< PCR 插入次数
};

// -----------------------------------------------------------------------------
// Muxer
// -----------------------------------------------------------------------------

/// PS / TS 复用器。
///
/// 典型用法：
/// ```cpp
/// MuxerConfig cfg;                          // 默认 TS
/// std::error_code ec;
/// auto mx = Muxer::create(cfg,
///     [](span<const std::uint8_t> out) { sock.send(out.data(), out.size()); },
///     ec);
///
/// StreamConfig sc; sc.codec = Codec::h264;
/// int sid = -1;
/// std::error_code ec2 = mx->addStream(sc, sid);   // sid 由库分配
///
/// mx->write(sid, FrameFlag::key, pts, dts, annexb_frame);      // 一帧一次
/// ```
///
/// @note 本类不可拷贝。**非线程安全**：同一对象的 write/addStream 必须在同一线程调用。
///
/// @warning **回调对象的生命周期**
///  构造时传入的 `OutputCallback` 会被**拷贝并持有**到本对象销毁。
///  lambda 捕获的对象（尤其 `[&]` 捕获的局部变量）必须活到 Muxer 销毁，
///  否则输出时会访问已释放内存。
///
///  正确做法：捕获 `shared_ptr`、或捕获值、或把回调对象定义在 Muxer 之外。
///
/// @warning **析构不回调**
///  `~Muxer()` 不触发输出回调、不隐式 flush。需要交出残留数据请显式调 `flush()`。
///  理由：析构期间触发回调，调用方可能在回调里访问正在析构的对象。
class Muxer {
public:
    /// 输出回调。参数是一段可直接发送的字节，回调返回后失效。
    using OutputCallback = std::function<void(span<const std::uint8_t>)>;

    /// 创建复用器。
    ///
    /// @param cfg     配置（`container` 必须明确为 ps 或 ts）
    /// @param on_out  输出回调（必填）
    /// @param ec      失败原因
    [[nodiscard]] static std::unique_ptr<Muxer> create(const MuxerConfig& cfg,
                                                       OutputCallback on_out,
                                                       std::error_code& ec);

    ~Muxer();

    Muxer(Muxer&&) noexcept;
    Muxer& operator=(Muxer&&) noexcept;
    Muxer(const Muxer&)            = delete;
    Muxer& operator=(const Muxer&) = delete;

    /// 登记一条 ES 流。
    ///
    /// @param[in]  cfg 流配置
    /// @param[out] ec  失败原因（capacity_exceeded / unsupported_codec /
    ///                 invalid_argument）
    /// @return 成功返回库内流索引（>= 0），失败返回 `std::nullopt`
    ///
    /// @note 必须在第一次 `write` 之前调用完毕。TS 的 PAT/PMT 在第一次 write
    ///       时按当时的流表生成；之后再 addStream 会触发 PMT 版本号 +1 并重发。
    ///
    /// @par 为什么用 optional 而不是出参
    ///  本库是 C++17、不考虑 C 兼容，出参 `int&` 会让失败时的值处于"未定义"
    ///  状态，调用方容易忘记检查。optional 把"有没有值"编码进类型里。
    [[nodiscard]] std::optional<int> addStream(const StreamConfig& cfg,
                                               std::error_code& ec);

    /// 写入一帧 ES。
    ///
    /// @param stream_id  `addStream` 返回的索引
    /// @param flags      帧标志（key / aud / config）
    /// @param pts        显示时间戳（90 kHz）；kNoTimestamp 表示无
    /// @param dts        解码时间戳（90 kHz）；等于 pts 表示无重排
    /// @param es         帧数据：H.26x 为 AnnexB，AAC 见 StreamConfig::add_adts
    /// @return 0 成功；stream_not_found / malformed / invalid_argument
    ///
    /// @note 视频帧若首部没有 AUD 且 `flags` 未含 `aud`，本库**不**主动插入
    ///       （避免与调用方重复）；需要 AUD 请在 flags 里告知，由实现按容器规范补。
    std::error_code write(int stream_id,
                          FrameFlag flags,
                          std::int64_t pts,
                          std::int64_t dts,
                          span<const std::uint8_t> es);

    /// 冲刷内部残留（TS 补最后一个包的 adaptation field、PS 关闭当前 pack），
    /// 并把 PCR/PSI 的待发定时器处理掉。流结束时调用一次。
    std::error_code flush();

    /// 统计快照。
    [[nodiscard]] const MuxStats& stats() const noexcept;

    /// 已登记的流数量。
    [[nodiscard]] std::size_t streamCount() const noexcept;

private:
    struct Impl;
    explicit Muxer(std::unique_ptr<Impl> impl) noexcept;

    std::unique_ptr<Impl> impl_;
};

} // namespace mpegmech
