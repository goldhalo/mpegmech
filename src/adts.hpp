// =============================================================================
//  adts.hpp —— ADTS 头（内部）
// =============================================================================
//  PS 里的 AAC 必须带 ADTS 头（7 字节），否则平台解不出来。
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mpegmech {
namespace detail {

using Bytes = std::vector<std::uint8_t>;

/// 从 AudioSpecificConfig（ASC）解析出的参数。
struct AacConfig {
    std::uint8_t  profile        = 2;   ///< audioObjectType（LC = 2）
    std::uint8_t  sampling_index = 4;   ///< 44100 Hz
    std::uint8_t  channel_config = 2;   ///< 立体声
    bool          valid          = false;
};

/// 解析 AudioSpecificConfig。
///
/// ASC 结构（GB28181 / ISO 14496-3）：
///   audioObjectType       5 bit
///   samplingFrequencyIndex 4 bit
///   channelConfiguration  4 bit
///
/// @param asc 至少 2 字节
[[nodiscard]] AacConfig parse_asc(const std::uint8_t* asc, std::size_t len) noexcept;

/// 采样率索引 → 实际采样率（Hz）。索引非法返回 0。
[[nodiscard]] std::uint32_t sampling_rate_of(std::uint8_t index) noexcept;

/// 写 7 字节 ADTS 头（无 CRC）。
///
/// @param frame_len 含 ADTS 头的总长度（= 7 + 裸帧长度）
/// @return false = 帧长超过 ADTS 能表达的上限（13 位，8183 字节）。
///
/// @note 失败时**不会**往 `out` 写任何字节。以前是静默钳位到 0x1FFF ——
///       那会得到一个"同步字合法、长度撒谎"的头，下游按它切帧从此错位，
///       比干脆没有头更难查。
[[nodiscard]] bool build_adts_header(Bytes& out, const AacConfig& cfg,
                                     std::size_t frame_len);

} // namespace detail
} // namespace mpegmech
