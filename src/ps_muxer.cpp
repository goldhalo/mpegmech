// =============================================================================
//  ps_muxer.cpp —— MPEG-2 PS 复用
// =============================================================================
//  GB28181 要求的 PS 结构：
//
//    [PackHeader 00 00 01 BA]  14 字节，SCR + mux_rate
//    [SystemHeader 00 00 01 BB]  可选但建议带
//    [PSM 00 00 01 BC]          stream_type → stream_id 映射
//    [PES 00 00 01 E0] x N
//
//  ★ 关键：每个**关键帧前**要重发 PackHeader + SystemHeader + PSM。
//    平台侧靠这个定位"一帧的开始"，不做的话解出来是花屏。
// =============================================================================
#include "ps_muxer.hpp"

#include <cstring>
#include <vector>
#include <cstddef>
#include <cstdint>

namespace mpegmech {
namespace detail {

namespace {

void put16(Bytes& v, std::uint16_t x)
{
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
}

void put32(Bytes& v, std::uint32_t x)
{
    v.push_back(static_cast<std::uint8_t>(x >> 24));
    v.push_back(static_cast<std::uint8_t>((x >> 16) & 0xFF));
    v.push_back(static_cast<std::uint8_t>((x >> 8) & 0xFF));
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
}

/// 写 PTS/DTS 的 5 字节编码。
///
/// @param prefix 0b0010 只有 PTS / 0b0011 PTS+DTS / 0b0001 只有 DTS
void put_timestamp(Bytes& v, std::uint8_t prefix, std::int64_t ts)
{
    const std::uint64_t t = static_cast<std::uint64_t>(ts) & 0x1FFFFFFFFull;
    v.push_back(static_cast<std::uint8_t>((prefix << 4) | ((t >> 29) & 0x0E) | 0x01));
    v.push_back(static_cast<std::uint8_t>((t >> 22) & 0xFF));
    v.push_back(static_cast<std::uint8_t>(((t >> 14) & 0xFE) | 0x01));
    v.push_back(static_cast<std::uint8_t>((t >> 7) & 0xFF));
    v.push_back(static_cast<std::uint8_t>(((t << 1) & 0xFE) | 0x01));
}

} // namespace

// =============================================================================
//  Pack Header（MPEG-2，14 字节）
// =============================================================================

void build_pack_header(Bytes& out, std::int64_t scr_90k, std::uint32_t mux_rate)
{
    out.push_back(0x00);
    out.push_back(0x00);
    out.push_back(0x01);
    out.push_back(0xBA);

    // '01' + SCR[32..30] + marker + SCR[29..15] + marker + SCR[14..0] + marker + ext(9) + marker
    const std::uint64_t scr = static_cast<std::uint64_t>(scr_90k) & 0x7FFFFFFFFull;
    const std::uint64_t base = scr;

    out.push_back(static_cast<std::uint8_t>(0x44 | ((base >> 27) & 0x38) | ((base >> 28) & 0x03)));
    out.push_back(static_cast<std::uint8_t>((base >> 20) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(((base >> 12) & 0xF8) | 0x04 | ((base >> 13) & 0x03)));
    out.push_back(static_cast<std::uint8_t>((base >> 5) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(((base << 3) & 0xF8) | 0x04 | 0x03));
    out.push_back(0x01);                                   // SCR extension 低 8 位

    // mux_rate：22 位，单位 50 bytes/s
    out.push_back(static_cast<std::uint8_t>(0x03 | ((mux_rate >> 15) & 0x7F)));
    out.push_back(static_cast<std::uint8_t>((mux_rate >> 7) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(((mux_rate << 1) & 0xFE) | 0x01));

    out.push_back(0xF8);                                   // pack_stuffing_length = 0
}

// =============================================================================
//  System Header（0x000001BB）
// =============================================================================

void build_system_header(Bytes& out, std::uint32_t mux_rate,
                         const std::vector<SystemStreamEntry>& streams)
{
    out.push_back(0x00);
    out.push_back(0x00);
    out.push_back(0x01);
    out.push_back(0xBB);
    const std::size_t len_pos = out.size();                // 长度字段的**起始**位置
    out.push_back(0);                                      // 长度占位（高字节）
    out.push_back(0);                                      // 长度占位（低字节）

    // rate_bound（22 位）+ marker
    out.push_back(static_cast<std::uint8_t>(0x80 | ((mux_rate >> 15) & 0x7F)));
    out.push_back(static_cast<std::uint8_t>((mux_rate >> 7) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(((mux_rate << 1) & 0xFE) | 0x01));

    out.push_back(static_cast<std::uint8_t>(streams.size()));  // audio_bound
    out.push_back(0xE0);                                   // fixed_flag=1, CSPS_flag=1
    out.push_back(0xFF);                                   // video_bound
    out.push_back(0x7F);                                   // packet_rate_restriction

    for (const auto& s : streams) {
        out.push_back(s.stream_id);
        out.push_back(static_cast<std::uint8_t>(0xC0 | (s.bs_buffer_bound >> 8)));
        out.push_back(static_cast<std::uint8_t>(s.bs_buffer_bound & 0xFF));
    }

    // 长度字段 = 它自己之后的字节数（不含这 2 字节）
    const std::size_t body_len = out.size() - len_pos - 2;
    out[len_pos]     = static_cast<std::uint8_t>((body_len >> 8) & 0xFF);
    out[len_pos + 1] = static_cast<std::uint8_t>(body_len & 0xFF);
}

// =============================================================================
//  PSM（0x000001BC）
// =============================================================================

void build_psm(Bytes& out, const std::vector<PsmStreamEntry>& streams,
               std::uint16_t version)
{
    out.push_back(0x00);
    out.push_back(0x00);
    out.push_back(0x01);
    out.push_back(0xBC);
    const std::size_t len_pos = out.size();                // 长度字段的起始位置
    out.push_back(0);
    out.push_back(0);

    // current_next_indicator(1) + reserved(2) + program_stream_map_version(5)
    out.push_back(static_cast<std::uint8_t>(0xE0 | (version & 0x1F)));
    // reserved(7) '1111111' + marker_bit(1) '1'
    out.push_back(0xFF);
    // ★ program_stream_info_length 是 **16 位**，必须写 2 个字节。
    //   早先只写了 1 个字节，导致后面所有字段错位 —— ffmpeg 把 AAC 的
    //   stream_type(0x0F) 读成了别的值，认成 mp3/mp2。
    out.push_back(0x00);
    out.push_back(0x00);

    // 元素流映射
    //
    // ★ elementary_stream_map_length 是**映射区的字节数**，不是流的条数。
    //   每条 4 字节（stream_type + elementary_stream_id + info_length）。
    //   早先按条数写（1 条 → 0x0001），解封装侧 `remaining >= 4` 永远不成立，
    //   整张 PSM 被当成空表跳过 —— 表面看没问题（视频靠 0xE0 猜、AAC 靠
    //   ADTS 同步字猜），实际 G.711 这种只能靠 PSM 认的流就彻底认不出来。
    const std::size_t map_bytes = streams.size() * 4;
    out.push_back(static_cast<std::uint8_t>((map_bytes >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(map_bytes & 0xFF));

    for (const auto& s : streams) {
        out.push_back(s.stream_type);
        out.push_back(s.elementary_stream_id);
        out.push_back(0);                                  // info_length 高
        out.push_back(0);                                  // info_length 低
    }

    // CRC32 占位（GB28181 平台多数不校验，但字段要在）
    put32(out, 0);

    const std::size_t body_len = out.size() - len_pos - 2;
    out[len_pos]     = static_cast<std::uint8_t>((body_len >> 8) & 0xFF);
    out[len_pos + 1] = static_cast<std::uint8_t>(body_len & 0xFF);
}

// =============================================================================
//  PES（视频 0xE0..0xEF）
// =============================================================================

void build_pes_header(Bytes& out, std::uint8_t stream_id,
                      std::int64_t pts, std::int64_t dts,
                      std::size_t payload_len, bool explicit_len)
{
    out.push_back(0x00);
    out.push_back(0x00);
    out.push_back(0x01);
    out.push_back(stream_id);

    const bool has_dts = (dts != pts);
    // PES_packet_length = 3（flags 那三字节）+ 5（PTS）+ (5 DTS) + payload
    const std::size_t hdr_after_len = 3 + 5 + (has_dts ? 5 : 0);
    if (explicit_len) {
        const std::size_t total = hdr_after_len + payload_len;
        put16(out, static_cast<std::uint16_t>(total > 0xFFFF ? 0 : total));
    } else {
        put16(out, 0);                                     // 0 = 不定长
    }

    out.push_back(0x80);                                   // '10' + PES_scrambling=0
    out.push_back(has_dts ? 0xC0 : 0x80);                  // PTS_DTS_flags
    out.push_back(static_cast<std::uint8_t>(5 + (has_dts ? 5 : 0)));

    put_timestamp(out, has_dts ? 0x03 : 0x02, pts);
    if (has_dts) {
        put_timestamp(out, 0x01, dts);
    }
}

} // namespace detail
} // namespace mpegmech
