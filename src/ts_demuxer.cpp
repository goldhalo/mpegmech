// =============================================================================
//  ts_demuxer.cpp —— TS 解封装实现
// =============================================================================
#include "ts_demuxer.hpp"

#include <algorithm>
#include <cstring>
#include <vector>
#include <map>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace mpegmech {
namespace detail {

namespace {

std::uint16_t get16(const std::uint8_t* p) noexcept
{
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

std::int64_t get_timestamp(const std::uint8_t* p) noexcept
{
    return (static_cast<std::int64_t>(p[0] & 0x0E) << 29) |
           (static_cast<std::int64_t>(p[1]) << 22) |
           (static_cast<std::int64_t>(p[2] & 0xFE) << 14) |
           (static_cast<std::int64_t>(p[3]) << 7) |
           (static_cast<std::int64_t>(p[4] & 0xFE) >> 1);
}

} // namespace

// =============================================================================
//  feed
// =============================================================================

void TsDemuxer::feed(const std::uint8_t* data, std::size_t len,
                     std::vector<Frame>& out)
{
    if (data != nullptr && len != 0) {
        m_buf.insert(m_buf.end(), data, data + len);
    }

    // ---- 同步：找连续 3 个 0x47（间隔 188）----
    if (m_state == TsSyncState::searching) {
        bool found = false;
        while (m_buf.size() - m_scan >= kTsPacketSize * 3) {
            const std::uint8_t* p = m_buf.data() + m_scan;
            if (p[0] == 0x47 &&
                p[kTsPacketSize] == 0x47 &&
                p[kTsPacketSize * 2] == 0x47) {
                found = true;
                break;
            }
            ++m_scan;
            ++m_skipped;
        }
        if (!found) {
            // 只保留可能构成同步点的尾巴
            const std::size_t keep = kTsPacketSize * 3;
            if (m_buf.size() > keep) {
                const std::size_t drop = m_buf.size() - keep;
                m_buf.erase(m_buf.begin(), m_buf.begin() + static_cast<std::ptrdiff_t>(drop));
                m_scan = 0;
            }
            return;
        }
        m_state = TsSyncState::waiting_psi;
    }

    // ---- 按 188 切包 ----
    while (m_buf.size() - m_scan >= kTsPacketSize) {
        const std::uint8_t* pkt = m_buf.data() + m_scan;

        if (pkt[0] != 0x47) {
            // 失步：回到搜索态，从当前位置往后找
            ++m_resyncs;
            m_state = TsSyncState::searching;
            ++m_scan;
            ++m_skipped;
            // 重新触发同步搜索
            while (m_buf.size() - m_scan >= kTsPacketSize * 3) {
                const std::uint8_t* q = m_buf.data() + m_scan;
                if (q[0] == 0x47 && q[kTsPacketSize] == 0x47 &&
                    q[kTsPacketSize * 2] == 0x47) {
                    m_state = (m_programs.empty() ? TsSyncState::waiting_psi
                                                  : TsSyncState::ready);
                    break;
                }
                ++m_scan;
                ++m_skipped;
            }
            if (m_state == TsSyncState::searching) { break; }
            continue;
        }

        parse_packet(pkt, out);
        m_scan += kTsPacketSize;
    }

    // 压缩已消费部分
    if (m_scan > 0) {
        if (m_scan >= m_buf.size()) {
            m_buf.clear();
            m_scan = 0;
        } else if (m_scan > m_buf.size() / 2) {
            m_buf.erase(m_buf.begin(), m_buf.begin() + static_cast<std::ptrdiff_t>(m_scan));
            m_scan = 0;
        }
    }
}

// =============================================================================
//  单包处理
// =============================================================================

void TsDemuxer::parse_packet(const std::uint8_t* pkt, std::vector<Frame>& out)
{
    const bool    pusi      = (pkt[1] & 0x40) != 0;
    const std::uint16_t pid = static_cast<std::uint16_t>(((pkt[1] & 0x1F) << 8) | pkt[2]);
    const std::uint8_t afc  = (pkt[3] >> 4) & 0x03;   // adaptation_field_control

    std::size_t off = 4;

    // adaptation field
    if (afc == 0x02 || afc == 0x03) {
        if (off >= kTsPacketSize) { return; }
        const std::uint8_t af_len = pkt[off];
        off += 1 + af_len;
        if (off > kTsPacketSize) { ++m_bad; return; }
    }

    // 没有 payload（afc == 0x02）
    if (afc == 0x02) {
        if (pid == 0x0000 || m_programs.find(pid) != m_programs.end()) { ++m_bad; }
        return;
    }

    if (off >= kTsPacketSize) { return; }
    const std::uint8_t* payload = pkt + off;
    const std::size_t   plen    = kTsPacketSize - off;

    // ---- PAT ----
    if (pid == 0x0000) {
        parse_psi(payload, plen, 0);
        return;
    }

    // ---- PMT（多节目时每个节目一个 PMT PID）----
    if (m_programs.find(pid) != m_programs.end()) {
        parse_psi(payload, plen, pid);
        return;
    }

    // ---- 其它 PID：还不在流表里就跳过 ----
    if (find_es(pid) == nullptr) { return; }

    parse_pes(pid, payload, plen, pusi, out);
}

const TsEsInfo* TsDemuxer::find_es(std::uint16_t pid) const noexcept
{
    for (const auto& e : m_streams) {
        if (e.pid == pid) { return &e; }
    }
    return nullptr;
}

// =============================================================================
//  PSI（PAT / PMT）
// =============================================================================

void TsDemuxer::parse_psi(const std::uint8_t* payload, std::size_t len, std::uint16_t pid)
{
    if (len < 1) { return; }
    const std::size_t pointer = payload[0];
    std::size_t off = 1 + pointer;
    if (off + 3 > len) { return; }

    const std::uint8_t table_id = payload[off];
    const std::size_t  sec_len  = static_cast<std::size_t>(
        ((payload[off + 1] & 0x0F) << 8) | payload[off + 2]);
    if (off + 3 + sec_len > len) { return; }

    const std::uint8_t* sec = payload + off;
    const std::size_t   end = 3 + sec_len - 4;      // 去掉 CRC32

    if (pid == 0x0000) {
        if (table_id != 0x00) { return; }
        // PAT：8 字节头（table_id / len / tsid / version / sec_num / last_sec），
        // 之后每个节目 4 字节：program_number(2) + PMT_PID(2)。
        //
        // 一次 PAT 就是当前完整节目表，所以整张重建 —— 多节目流里不同节目的
        // PMT 各占一个 PID，早先只记最后一个，等于只解最后一套节目。
        std::map<std::uint16_t, std::uint16_t> progs;
        for (std::size_t p = 8; p + 4 <= end; p += 4) {
            const std::uint16_t prog = get16(sec + p);
            const std::uint16_t pmt  = static_cast<std::uint16_t>(
                ((sec[p + 2] & 0x1F) << 8) | sec[p + 3]);
            if (prog != 0) { progs[pmt] = prog; }
        }
        if (progs.empty()) { return; }

        // 已经不存在的节目，它的流表项要清掉，否则会继续解到死掉的 PID
        for (auto it = m_streams.begin(); it != m_streams.end();) {
            bool alive = false;
            for (const auto& kv : progs) {
                if (kv.second == it->program) { alive = true; break; }
            }
            it = alive ? it + 1 : m_streams.erase(it);
        }
        m_programs.swap(progs);
        if (m_state == TsSyncState::waiting_psi) { m_state = TsSyncState::ready; }
        return;
    }

    // PMT：头部固定 12 字节
    //   [0]table_id [1..2]section_length [3..4]program_number [5]version
    //   [6]section_number [7]last_section_number [8..9]PCR_PID
    //   [10..11]program_info_length
    // ★ program_info_length 在偏移 10，不是 12 —— 早先从 12 读，直接跳过了
    //   整条 PMT 的 ES 循环，导致"同步上了但认不出流"。
    if (table_id != 0x02) { return; }

    const std::uint16_t program = m_programs[pid];

    std::size_t p = 10;
    if (p + 2 > end) { return; }
    const std::size_t prog_info_len = static_cast<std::size_t>(
        ((sec[p] & 0x0F) << 8) | sec[p + 1]);
    p += 2 + prog_info_len;

    // 只替换本节目下的流，别的节目不动
    m_streams.erase(std::remove_if(m_streams.begin(), m_streams.end(),
                                   [program](const TsEsInfo& e) {
                                       return e.program == program;
                                   }),
                    m_streams.end());

    while (p + 5 <= end) {
        const std::uint8_t  st  = sec[p];
        const std::uint16_t ep  = static_cast<std::uint16_t>(
            ((sec[p + 1] & 0x1F) << 8) | sec[p + 2]);
        const std::size_t   ilen = static_cast<std::size_t>(
            ((sec[p + 3] & 0x0F) << 8) | sec[p + 4]);
        m_streams.push_back({st, ep, program});
        p += 5 + ilen;
    }

    if (m_state != TsSyncState::ready && !m_streams.empty()) {
        m_state = TsSyncState::ready;
    }
}

// =============================================================================
//  PES 累积
// =============================================================================

void TsDemuxer::parse_pes(std::uint16_t pid, const std::uint8_t* payload,
                          std::size_t len, bool pusi, std::vector<Frame>& out)
{
    auto& acc = m_pes[pid];

    if (pusi) {
        // 新的 PES 开始：把上一条交出去（它没有已知长度，只能靠下一条 PUSI 收尾）
        if (acc.active && !acc.data.empty()) {
            emit_pes(pid, out);
        }
        acc.data.clear();
        acc.active = true;
        if (len > 0) {
            acc.data.insert(acc.data.end(), payload, payload + len);
        } else {
            acc.active = false;
            return;
        }
    } else if (acc.active) {
        acc.data.insert(acc.data.end(), payload, payload + len);
    } else {
        return;   // 没在收集，丢弃（从流中间接入时的首个残包）
    }

    // PES_packet_length != 0 时可以先收完先出，不必等下一条 PUSI
    if (acc.data.size() >= 6) {
        const std::size_t pes_len = static_cast<std::size_t>(
            (static_cast<std::uint16_t>(acc.data[4]) << 8) | acc.data[5]);
        if (pes_len != 0 && acc.data.size() >= 6 + pes_len) {
            emit_pes(pid, out);
        }
    }
}

void TsDemuxer::emit_pes(std::uint16_t pid, std::vector<Frame>& out)
{
    auto it = m_pes.find(pid);
    if (it == m_pes.end()) { return; }

    auto& acc = it->second;
    if (!acc.active || acc.data.size() < 9) {
        acc.active = false;
        acc.data.clear();
        return;
    }

    const std::uint8_t* p = acc.data.data();
    // 校验 PES 起始码
    if (p[0] != 0 || p[1] != 0 || p[2] != 1) {
        acc.active = false;
        acc.data.clear();
        ++m_bad;
        return;
    }

    // PES_packet_length 非 0 时按长度裁剪（TS 的最后一个包会用 AF 填充到 188，
    // payload 里可能带 stuffing，直接取到尾会把垃圾塞进 ES）
    const std::size_t pes_len = static_cast<std::size_t>(
        (static_cast<std::uint16_t>(p[4]) << 8) | p[5]);
    std::size_t avail = acc.data.size();
    if (pes_len != 0) {
        const std::size_t total = 6 + pes_len;
        avail = (avail < total) ? avail : total;
    }

    const std::uint8_t hdr_len = p[8];
    if (9u + hdr_len > avail) {
        acc.active = false;
        acc.data.clear();
        ++m_bad;
        return;
    }

    std::int64_t pts = kNoTimestamp;
    std::int64_t dts = kNoTimestamp;
    if (hdr_len >= 5) {
        const int pts_dts = (p[7] >> 6) & 0x03;
        if (pts_dts == 0x02 || pts_dts == 0x03) {
            pts = get_timestamp(p + 9);
            if (pts_dts == 0x03 && hdr_len >= 10) {
                dts = get_timestamp(p + 14);
            }
        }
    }

    Frame f;
    f.pid  = pid;
    f.pts  = pts;
    f.dts  = (dts == kNoTimestamp) ? pts : dts;
    if (const TsEsInfo* es = find_es(pid)) {
        f.stream_type = es->stream_type;
        f.program     = es->program;
    }
    f.payload.assign(acc.data.begin() + static_cast<std::ptrdiff_t>(9 + hdr_len),
                     acc.data.begin() + static_cast<std::ptrdiff_t>(avail));
    if (!f.payload.empty()) {
        out.push_back(std::move(f));
    }

    acc.active = false;
    acc.data.clear();
}

// =============================================================================
//  flush / reset
// =============================================================================

void TsDemuxer::flush(std::vector<Frame>& out)
{
    // 把每个 PID 的残留都交出去（flush 之后流结束，剩下的半条 PES 也算一帧）
    for (auto& kv : m_pes) {
        if (kv.second.active && !kv.second.data.empty()) {
            emit_pes(kv.first, out);
        }
    }
    m_pes.clear();
}

void TsDemuxer::reset()
{
    m_state = TsSyncState::searching;
    m_buf.clear();
    m_scan = 0;
    m_programs.clear();
    m_streams.clear();
    m_pes.clear();
    m_skipped = 0;
    m_resyncs = 0;
    m_bad = 0;
}

} // namespace detail
} // namespace mpegmech
