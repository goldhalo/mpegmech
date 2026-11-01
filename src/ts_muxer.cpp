// =============================================================================
//  ts_muxer.cpp —— TS 组包实现
// =============================================================================
#include "ts_muxer.hpp"

#include <cstring>

namespace mpegmech {
namespace detail {

namespace {

void put16(Bytes& v, std::uint16_t x)
{
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
}

} // namespace

std::uint32_t crc32_mpeg2(const std::uint8_t* data, std::size_t len) noexcept
{
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<std::uint32_t>(data[i]) << 24;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
    }
    return crc;
}

void write_ts_packets(Bytes& out, std::uint16_t pid, const std::uint8_t* data,
                      std::size_t len, bool pusi, std::uint8_t& cc)
{
    std::size_t off = 0;
    bool first = true;
    while (off < len || (len == 0 && first)) {
        const std::size_t remain = len - off;

        // ---- 包头 4 字节 ----
        out.push_back(0x47);
        out.push_back(static_cast<std::uint8_t>(
            ((pusi && first) ? 0x40 : 0x00) | ((pid >> 8) & 0x1F)));
        out.push_back(static_cast<std::uint8_t>(pid & 0xFF));

        const std::size_t room = kTsPacketSize - 4;
        if (remain >= room) {
            // 满包
            out.push_back(static_cast<std::uint8_t>(0x10 | (cc & 0x0F)));
            out.insert(out.end(), data + off, data + off + room);
            off += room;
        } else {
            // 不足 184：用 adaptation field 填充
            //
            //   [len 1][flags 1][stuffing N]
            //   总共占 (room - remain) 字节
            const std::size_t stuff = room - remain;
            out.push_back(static_cast<std::uint8_t>(0x30 | (cc & 0x0F)));  // adaptation + payload
            if (stuff == 1) {
                out.push_back(0x00);             // 只有长度字节，没有 flags
            } else {
                out.push_back(static_cast<std::uint8_t>(stuff - 1));
                out.push_back(0x00);             // flags
                for (std::size_t i = 0; i < stuff - 2; ++i) {
                    out.push_back(0xFF);         // stuffing
                }
            }
            out.insert(out.end(), data + off, data + len);
            off = len;
        }

        cc = static_cast<std::uint8_t>((cc + 1) & 0x0F);
        first = false;
        if (len == 0) { break; }
    }
}

void build_pat(Bytes& out, std::uint16_t transport_stream_id,
               const std::vector<std::pair<std::uint16_t, std::uint16_t>>& programs,
               std::uint8_t version, std::uint8_t& cc)
{
    Bytes sec;
    sec.push_back(0x00);                        // pointer_field

    // table_id = 0x00（PAT）
    sec.push_back(0x00);

    // section_syntax_indicator(1) + '0' + reserved(2) + section_length(12)
    const std::size_t body_len = 5 + programs.size() * 4 + 4;   // 含 CRC
    sec.push_back(static_cast<std::uint8_t>(0xB0 | ((body_len >> 8) & 0x0F)));
    sec.push_back(static_cast<std::uint8_t>(body_len & 0xFF));

    put16(sec, transport_stream_id);

    // reserved(2) + version(5) + current_next(1)
    sec.push_back(static_cast<std::uint8_t>(0xC1 | ((version & 0x1F) << 1)));
    sec.push_back(0x00);                        // section_number
    sec.push_back(0x00);                        // last_section_number

    for (const auto& p : programs) {
        put16(sec, p.first);                    // program_number
        sec.push_back(static_cast<std::uint8_t>(0xE0 | ((p.second >> 8) & 0x1F)));
        sec.push_back(static_cast<std::uint8_t>(p.second & 0xFF));
    }

    const std::uint32_t crc = crc32_mpeg2(sec.data() + 1, sec.size() - 1);
    sec.push_back(static_cast<std::uint8_t>((crc >> 24) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>((crc >> 16) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>(crc & 0xFF));

    write_ts_packets(out, 0x0000, sec.data(), sec.size(), true, cc);
}

void build_pmt(Bytes& out, std::uint16_t program_number, std::uint16_t pmt_pid,
               std::uint16_t pcr_pid, const std::vector<TsEsEntry>& es,
               std::uint8_t version, std::uint8_t& cc)
{
    Bytes sec;
    sec.push_back(0x00);                        // pointer_field
    sec.push_back(0x02);                        // table_id = PMT

    const std::size_t body_len = 9 + es.size() * 5 + 4;
    sec.push_back(static_cast<std::uint8_t>(0xB0 | ((body_len >> 8) & 0x0F)));
    sec.push_back(static_cast<std::uint8_t>(body_len & 0xFF));

    put16(sec, program_number);
    sec.push_back(static_cast<std::uint8_t>(0xC1 | ((version & 0x1F) << 1)));
    sec.push_back(0x00);
    sec.push_back(0x00);

    // reserved(3) + PCR_PID(13)
    sec.push_back(static_cast<std::uint8_t>(0xE0 | ((pcr_pid >> 8) & 0x1F)));
    sec.push_back(static_cast<std::uint8_t>(pcr_pid & 0xFF));

    // reserved(4) + program_info_length(12) = 0
    sec.push_back(0xF0);
    sec.push_back(0x00);

    for (const auto& e : es) {
        sec.push_back(e.stream_type);
        sec.push_back(static_cast<std::uint8_t>(0xE0 | ((e.elementary_pid >> 8) & 0x1F)));
        sec.push_back(static_cast<std::uint8_t>(e.elementary_pid & 0xFF));
        sec.push_back(0xF0);                    // reserved(4) + ES_info_length 高 4 位
        sec.push_back(0x00);
    }

    const std::uint32_t crc = crc32_mpeg2(sec.data() + 1, sec.size() - 1);
    sec.push_back(static_cast<std::uint8_t>((crc >> 24) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>((crc >> 16) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
    sec.push_back(static_cast<std::uint8_t>(crc & 0xFF));

    write_ts_packets(out, pmt_pid, sec.data(), sec.size(), true, cc);
}

} // namespace detail
} // namespace mpegmech
