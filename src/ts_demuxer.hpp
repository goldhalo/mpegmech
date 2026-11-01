// =============================================================================
//  ts_demuxer.hpp —— TS 解封装（内部）
// =============================================================================
//  TS 跟 PS 的本质差别在这里体现得最清楚：
//
//    PS：起始码自解释（00 00 01 E0 就是视频 PES），扫到就能干。
//    TS：188 包只是"容器"，要知道哪个 PID 是视频，**必须等 PMT**。
//        所以从流中间接入时，要等一个 PSI 周期（通常 100~500ms）。
// =============================================================================
#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include <mpegmech/error.hpp>
#include <mpegmech/types.hpp>   // kNoTimestamp、StreamType

namespace mpegmech {
namespace detail {

/// TS 包大小。
constexpr std::size_t kTsPacketSize = 188;

/// 同步状态。
enum class TsSyncState {
    searching,     ///< 还没对齐 188 包边界
    waiting_psi,   ///< 包对齐了，但还没收到 PAT/PMT
    ready,         ///< 有 PAT/PMT，可以解 PES
};

/// 一条 ES 的信息（从 PMT 得到）。
struct TsEsInfo {
    std::uint8_t  stream_type = 0;
    std::uint16_t pid         = 0;
};

/// TS 解复用的核心状态机。
class TsDemuxer {
public:
    /// 一帧解析结果。
    struct Frame {
        std::uint8_t              stream_type = 0;
        std::uint16_t             pid         = 0;
        std::int64_t              pts         = kNoTimestamp;
        std::int64_t              dts         = kNoTimestamp;
        std::vector<std::uint8_t> payload;    ///< PES 里的 ES 数据
    };

    /// 解析缓冲区里的数据，返回解析出的帧。
    ///
    /// @param data  输入字节（任意长度）
    /// @param len   长度
    /// @param out   解析出的帧（追加）
    /// @param ec    失败原因（waiting_psi 不是错误，用 state() 判断）
    void feed(const std::uint8_t* data, std::size_t len, std::vector<Frame>& out);

    /// 冲刷残留。
    void flush(std::vector<Frame>& out);

    /// 重置。
    void reset();

    [[nodiscard]] TsSyncState state() const noexcept { return m_state; }
    [[nodiscard]] const std::vector<TsEsInfo>& streams() const noexcept { return m_streams; }
    [[nodiscard]] std::uint64_t bytes_skipped() const noexcept { return m_skipped; }
    [[nodiscard]] std::uint64_t resyncs() const noexcept { return m_resyncs; }
    [[nodiscard]] std::uint64_t bad_packets() const noexcept { return m_bad; }
    [[nodiscard]] std::size_t buffered() const noexcept { return m_buf.size() - m_scan; }

private:
    void parse_packet(const std::uint8_t* pkt, std::vector<Frame>& out);
    void parse_psi(const std::uint8_t* payload, std::size_t len, std::uint16_t pid);
    void parse_pes(std::uint16_t pid, const std::uint8_t* payload, std::size_t len,
                   bool pusi, std::vector<Frame>& out);
    void emit_pes(std::uint16_t pid, std::vector<Frame>& out);

    TsSyncState m_state = TsSyncState::searching;

    std::vector<std::uint8_t> m_buf;      ///< 未处理的字节
    std::size_t               m_scan = 0; ///< 扫描位置

    // PSI
    std::uint16_t m_pmt_pid = 0;
    bool          m_have_pat = false;
    std::vector<TsEsInfo> m_streams;

    // 每个 PID 的 PES 累积
    struct PesAccum {
        std::vector<std::uint8_t> data;
        bool                      active = false;
    };
    std::map<std::uint16_t, PesAccum> m_pes;

    // 统计
    std::uint64_t m_skipped = 0;
    std::uint64_t m_resyncs = 0;
    std::uint64_t m_bad     = 0;
};

} // namespace detail
} // namespace mpegmech
