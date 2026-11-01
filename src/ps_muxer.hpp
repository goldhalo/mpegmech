// =============================================================================
//  ps_muxer.hpp —— PS 组包的内部接口
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

namespace mpegmech {
namespace detail {

using Bytes = std::vector<std::uint8_t>;

/// System Header 里的一条流条目。
struct SystemStreamEntry {
    std::uint8_t  stream_id = 0xE0;
    std::uint32_t bs_buffer_bound = 32 * 1024;   ///< 以 1 KiB 为单位
};

/// PSM 里的一条元素流映射。
struct PsmStreamEntry {
    std::uint8_t stream_type        = 0x1B;      ///< H.264
    std::uint8_t elementary_stream_id = 0xE0;
};

/// Pack Header（0x000001BA），MPEG-2 版本，14 字节。
void build_pack_header(Bytes& out, std::int64_t scr_90k, std::uint32_t mux_rate);

/// System Header（0x000001BB）。
void build_system_header(Bytes& out, std::uint32_t mux_rate,
                         const std::vector<SystemStreamEntry>& streams);

/// Program Stream Map（0x000001BC）。
void build_psm(Bytes& out, const std::vector<PsmStreamEntry>& streams,
               std::uint16_t version);

/// PES 头（不含 payload）。
///
/// @param explicit_len true 填真实长度；false 填 0（不定长，流式友好）
void build_pes_header(Bytes& out, std::uint8_t stream_id,
                      std::int64_t pts, std::int64_t dts,
                      std::size_t payload_len, bool explicit_len);

} // namespace detail
} // namespace mpegmech
