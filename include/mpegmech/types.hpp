// =============================================================================
//  mpegmech/types.hpp —— 公共类型
// =============================================================================
//  文件    : mpegmech/types.hpp
//  用途    : 容器种类、编码类型、帧标志、流/帧描述、时间基换算。
//  编码    : UTF-8
//
//  ── 设计说明 ────────────────────────────────────────────────────────────────
//  * `Container` 同时用于两处：**配置**里 `unknown` 表示"按流内容自动嗅探"；
//    **结果**里 `unknown` 表示"还没识别出来"。语义靠上下文区分，见各处注释。
//  * `Codec` 是本库的**内部统一编码标识**，与 ISO/IEC 13818-1 的 stream_type
//    解耦。PS 的 stream_id（0xE0..0xEF）与 TS 的 stream_type（0x1B / 0x24）
//    都在实现层映射到这张表，调用方不必关心。
//  * 时间戳单位统一为 **90 kHz**（MPEG 系统时钟），与 PTS/DTS 的 33 bit 一致。
//    换算函数用整数运算，不引入浮点误差。
// =============================================================================
#pragma once

#include <cstdint>
#include <string>

#include "mpegmech/error.hpp"
#include "mpegmech/span.hpp"

namespace mpegmech {

// -----------------------------------------------------------------------------
// 容器
// -----------------------------------------------------------------------------

/// 复用格式。
enum class Container : std::uint8_t {
    unknown = 0,  ///< 配置里：自动嗅探；结果里：未识别
    ps      = 1,  ///< MPEG-2 Program Stream（ISO/IEC 13818-1 系统层）
    ts      = 2,  ///< MPEG-2 Transport Stream
};

// -----------------------------------------------------------------------------
// 编码类型
// -----------------------------------------------------------------------------

/// ES 编码类型。
enum class Codec : std::uint8_t {
    unknown = 0,

    // ---- 视频 ----
    mpeg1_video,
    mpeg2_video,
    mpeg4_video,
    h264,        ///< 输出为 AnnexB（00 00 00 01 起始码）
    h265,        ///< 同上
    h266,        ///< 同上
    svac_video,
    avs3,

    // ---- 音频 ----
    aac,         ///< 输出形态见 StreamInfo::adts
    mp3,
    ac3,
    eac3,
    g711a,
    g711u,
    g722,
    g723,
    g729,
    opus,
    svac_audio,
};

/// 是否视频编码（用于分配 PID / stream_id 与选择 PCR 载体）。
[[nodiscard]] constexpr bool isVideo(Codec c) noexcept
{
    return c >= Codec::mpeg1_video && c <= Codec::avs3;
}

/// 是否音频编码。
[[nodiscard]] constexpr bool isAudio(Codec c) noexcept
{
    return c >= Codec::aac && c <= Codec::svac_audio;
}

/// 是否需要 AnnexB 起始码（H.26x 系列）。
[[nodiscard]] constexpr bool isAnnexB(Codec c) noexcept
{
    return c == Codec::h264 || c == Codec::h265 || c == Codec::h266;
}

// -----------------------------------------------------------------------------
// 帧标志
// -----------------------------------------------------------------------------

/// 帧标志（位掩码）。mux 时由调用方给出，demux 时由库填。
enum class FrameFlag : std::uint32_t {
    none       = 0,
    key        = 1u << 0,  ///< 关键帧（视频：IDR；音频恒视为 key）
    aud        = 1u << 1,  ///< ES 首部已带 AUD（H.26x）
    config     = 1u << 2,  ///< 参数集帧（SPS/PPS/VPS），非显示帧
    incomplete = 1u << 3,  ///< 帧不完整（重同步、流截断或超长被裁）
};

[[nodiscard]] constexpr FrameFlag operator|(FrameFlag a, FrameFlag b) noexcept
{
    return static_cast<FrameFlag>(static_cast<std::uint32_t>(a) |
                                  static_cast<std::uint32_t>(b));
}

[[nodiscard]] constexpr FrameFlag operator&(FrameFlag a, FrameFlag b) noexcept
{
    return static_cast<FrameFlag>(static_cast<std::uint32_t>(a) &
                                  static_cast<std::uint32_t>(b));
}

inline FrameFlag& operator|=(FrameFlag& a, FrameFlag b) noexcept
{
    a = a | b;
    return a;
}

/// 判断 `v` 是否含标志 `f`。
[[nodiscard]] constexpr bool hasFlag(FrameFlag v, FrameFlag f) noexcept
{
    return (v & f) != FrameFlag::none;
}

// -----------------------------------------------------------------------------
// 时间基
// -----------------------------------------------------------------------------

/// MPEG 系统时钟频率（PTS/DTS 单位）。
inline constexpr std::int32_t kClockHz = 90000;

/// "无时间戳"哨兵值（PES 里 PTS/DTS 可缺省）。
inline constexpr std::int64_t kNoTimestamp = INT64_MIN;

/// 毫秒 -> 90 kHz 时钟。
[[nodiscard]] std::int64_t msToClock(std::int64_t ms) noexcept;

/// 90 kHz 时钟 -> 毫秒。
[[nodiscard]] std::int64_t clockToMs(std::int64_t clock) noexcept;

// -----------------------------------------------------------------------------
// 流 / 帧描述
// -----------------------------------------------------------------------------

/// 一条 ES 流的属性。
///
/// @note `extradata` **指向库内部内存**，只在 `StreamCallback` 回调期间有效；
///       需要长期保存请自行拷贝。PS 场景通常为 nullptr（SPS/PPS 在码流里），
///       TS 场景来自 PMT 描述符或首个 PES 的 extradata。
struct StreamInfo {
    int                 stream_id = -1;        ///< 库内流索引（>= 0）
    Codec               codec     = Codec::unknown;
    std::uint16_t       program   = 0;         ///< TS: program_number；PS: 恒为 1
    std::uint16_t       pid       = 0;         ///< TS: PID；PS: 0
    bool                adts      = true;      ///< AAC: payload 是否带 ADTS 头
    span<const std::uint8_t> extradata{};      ///< 见上方生命周期说明
};

/// 一帧 ES 的描述。
struct FrameInfo {
    int           stream_id = -1;
    Codec         codec     = Codec::unknown;
    FrameFlag     flags     = FrameFlag::none;
    std::int64_t  pts       = kNoTimestamp;    ///< 90 kHz
    std::int64_t  dts       = kNoTimestamp;    ///< 90 kHz；等于 pts 时表示无 B 帧重排
    std::uint16_t program   = 0;
};

// -----------------------------------------------------------------------------
// 嗅探
// -----------------------------------------------------------------------------

/// 按内容猜测容器格式（PS / TS）。
///
/// 判据（任一命中即返回）：
///   * TS：缓冲内出现 **连续 3 个 0x47 且间隔 188 字节** —— 同步字节 + 包长双重约束，
///         裸数据里偶然撞上的概率极低；
///   * PS：出现 `00 00 01 BA`（pack_header_start_code）。
///
/// @return 识别不出来返回 `Container::unknown`（数据不足或不是 PS/TS）。
[[nodiscard]] Container sniffContainer(span<const std::uint8_t> data) noexcept;

/// 枚举名（日志用）。
[[nodiscard]] const char* toString(Container c) noexcept;
[[nodiscard]] const char* toString(Codec c) noexcept;

} // namespace mpegmech
