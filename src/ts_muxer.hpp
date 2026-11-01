// =============================================================================
//  ts_muxer.hpp —— TS 组包（内部）
// =============================================================================
//  TS 跟 PS 的本质差别：
//
//    PS：变长的 Pack + PES，靠起始码定位       —— 允许从流中间接入
//    TS：固定 188 字节包，靠 0x47 同步 + PAT/PMT 表  —— 必须等表才知道 PID 含义
//
//  所以 TS 的封装要先把 PAT/PMT 发出去，接收端才能解释后续的 PES。
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

namespace mpegmech {
namespace detail {

using Bytes = std::vector<std::uint8_t>;

/// TS 包大小。
constexpr std::size_t kTsPacketSize = 188;

/// MPEG-2 CRC32（多项式 0x04C11DB7，初值 0xFFFFFFFF，不取反）。
std::uint32_t crc32_mpeg2(const std::uint8_t* data, std::size_t len) noexcept;

/// 写一个 PAT 段（PID 0x0000）。
///
/// @param programs  program_number → PMT PID
void build_pat(Bytes& out, std::uint16_t transport_stream_id,
               const std::vector<std::pair<std::uint16_t, std::uint16_t>>& programs,
               std::uint8_t version, std::uint8_t& cc);

/// 一条 ES 描述（PMT 里的一行）。
struct TsEsEntry {
    std::uint8_t  stream_type  = 0x1B;   ///< H.264
    std::uint16_t elementary_pid = 0x0100;
};

/// 写一个 PMT 段。
void build_pmt(Bytes& out, std::uint16_t program_number, std::uint16_t pmt_pid,
               std::uint16_t pcr_pid, const std::vector<TsEsEntry>& es,
               std::uint8_t version, std::uint8_t& cc);

/// 把一段字节切成 188 字节包写出去。
///
/// @param pusi       第一包的 payload_unit_start_indicator 是否为 1
/// @param cc         该 PID 的 continuity_counter（函数内自增）
/// @param pad_last   最后一包不足 184 字节时，用 adaptation field 填充
///                   （PSI 段需要；PES 的最后一段不需要 —— 用 stuffing 也行）
void write_ts_packets(Bytes& out, std::uint16_t pid, const std::uint8_t* data,
                      std::size_t len, bool pusi, std::uint8_t& cc);

} // namespace detail
} // namespace mpegmech
