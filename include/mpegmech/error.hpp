// =============================================================================
//  mpegmech/error.hpp —— 错误码
// =============================================================================
//  文件    : mpegmech/error.hpp
//  用途    : 提供 mpegmech 统一的错误码体系（基于 std::error_code）。
//  编码    : UTF-8
//
//  ── 设计说明 ────────────────────────────────────────────────────────────────
//  * 与 rtpmech / netmech 保持一致：可预期的失败一律用 `std::error_code` 返回，
//    不抛异常（目标平台含嵌入式板卡，异常支持不一定可靠）。
//  * `MpegErrc` 通过 `std::error_code` 隐式构造（见下方 is_error_code_enum 特化），
//    调用方可以写：
//        std::error_code ec = MpegErrc::unsupported_container;
//        if (ec == MpegErrc::unsupported_container) { ... }
//
//  * 错误分四级，调用方按级决定处置方式：
//
//      L1 **调用方错误**（invalid_argument / stream_not_found / not_configured）
//         立即返回，**不改内部状态**。属于调用方的 bug，修了再调即可。
//
//      L2 **数据错误**（malformed / unsupported_codec）
//         丢掉这一个单元，计数后继续。流整体仍然可用。
//
//      L3 **失步**（sync_lost / buffer_overflow / waiting_psi）
//         库已自行处置（回搜索态、丢最旧数据、等 PSI），统计里有计数。
//         **不中断**，调用方可以直接继续 feed。
//
//         `waiting_psi` 严格说不算错误 —— 它是"TS 已对齐包边界，但还没
//         收到 PAT/PMT，不知道哪个 PID 是视频"。调用方看到它不用做什么，
//         继续喂就行；如果 500ms 后还是它，才说明流有问题。
//
//      L4 **致命**（out_of_memory / internal_error）
//         对象进入 **failed 状态**：后续所有 feed/write/flush 立刻返回
//         同一个错误码。**必须显式调 reset() 才能恢复**。
//
//         为什么不用"自动重置"：那会让调用方不知道发生过什么。也不用
//         "直接报废"：一次 bad_alloc 不该让整条链路重建对象。
// =============================================================================
#pragma once

#include <string>
#include <system_error>

namespace mpegmech {

// -----------------------------------------------------------------------------
// MpegErrc
// -----------------------------------------------------------------------------

/// mpegmech 错误码。
enum class MpegErrc {
    ok = 0,
    invalid_argument,         ///< 参数非法（空回调、码率/寄存器数为 0 等）
    not_configured,           ///< 未创建就调用 / 未 addStream 就 write
    unsupported_container,    ///< 无法识别为 PS 或 TS（嗅探失败）
    unsupported_codec,        ///< 当前容器不支持该 codec，或反之
    malformed,                ///< 结构非法（PES 长度越界、PMT 缺失、scrambling 置位等）
    sync_lost,                ///< 连续失步超过 ts_max_resync_bytes，本流已放弃
    buffer_overflow,          ///< 内部暂存达到上限，已按策略丢弃最旧数据
    stream_not_found,         ///< stream_id 不存在
    /// 流数量超上限（TS 单节目 16 路 / PS 32 路）。
    ///
    /// @warning **当前没有任何返回点**：`addStream()` 不检查流数上限，
    ///          这个错误码只是预留。`Muxer::create` 也不会因为它失败。
    capacity_exceeded,

    // ---- L3：失步类（不中断，统计可见）----
    waiting_psi,              ///< TS 包已对齐，但还没收到 PAT/PMT（继续喂即可）
    already_flushed,          ///< flush 之后又 feed/write

    // ---- L4：致命（对象进 failed 状态，要 reset() 才能继续）----
    out_of_memory,            ///< 分配失败
    internal_error,           ///< 内部不变量破坏（不该发生，发生了就是 bug）
};

/// mpegmech 的错误类别。
[[nodiscard]] const std::error_category& mpeg_category() noexcept;

/// 由 `MpegErrc` 构造 `std::error_code`。
[[nodiscard]] inline std::error_code make_error_code(MpegErrc e) noexcept
{
    return std::error_code(static_cast<int>(e), mpeg_category());
}

} // namespace mpegmech

// 让 `MpegErrc` 可以被隐式转换为 `std::error_code`。
namespace std {
template <>
struct is_error_code_enum<mpegmech::MpegErrc> : true_type {};
} // namespace std
