// =============================================================================
//  src/types.cpp —— 公共类型实现的工具函数
// =============================================================================
//  文件    : src/types.cpp
//  用途    : 时间基换算、容器嗅探、枚举名。
//  编码    : UTF-8
// =============================================================================

#include "mpegmech/types.hpp"

namespace mpegmech {
namespace {

/// TS 包长（188 = 4 字节头 + 184 字节 payload/adaptation）。
constexpr std::size_t kTsPacketBytes = 188;

/// 嗅探时最多扫描的字节数。超过这个长度还没找到同步点，说明不是 PS/TS，
/// 或者头部有一大段垃圾 —— 两种情况下继续扫描都没有意义。
constexpr std::size_t kSniffScanBytes = 64 * 1024;

/// 嗅探 TS 时要求的连续包数（与 DemuxerConfig 的默认值保持一致）。
constexpr std::size_t kSniffTsPackets = 3;

/// 判断 `data[off..]` 处是否是 TS 同步点。
bool isTsSyncAt(span<const std::uint8_t> data, std::size_t off) noexcept
{
    for (std::size_t n = 0; n < kSniffTsPackets; ++n) {
        const std::size_t pos = off + n * kTsPacketBytes;
        if (pos >= data.size() || data[pos] != 0x47) {
            return false;
        }
    }
    return true;
}

} // namespace

// -----------------------------------------------------------------------------
// 时间基
// -----------------------------------------------------------------------------

std::int64_t msToClock(std::int64_t ms) noexcept
{
    // 90 kHz / 1000 = 90。int64 下 ms 到 3.2e15 才溢出（约 10 万年），不设防。
    return ms * (kClockHz / 1000);
}

std::int64_t clockToMs(std::int64_t clock) noexcept
{
    // 向下取整（负数同样向负无穷），保证「同一毫秒内的帧归到同一档」。
    const std::int64_t q = clock / (kClockHz / 1000);
    return (clock % (kClockHz / 1000) != 0 && clock < 0) ? q - 1 : q;
}

// -----------------------------------------------------------------------------
// 嗅探
// -----------------------------------------------------------------------------

Container sniffContainer(span<const std::uint8_t> data) noexcept
{
    const std::size_t limit = data.size() < kSniffScanBytes ? data.size()
                                                            : kSniffScanBytes;

    // ---- 先判 TS ----
    // 判据更强（3 个 0x47 间隔 188），先判可以避免把 TS 误认成 PS。
    // 包边界必然落在前 188 字节内，所以只在前 188 个位置尝试。
    if (data.size() >= kTsPacketBytes * kSniffTsPackets) {
        const std::size_t tries = limit < kTsPacketBytes ? limit : kTsPacketBytes;
        for (std::size_t off = 0; off < tries; ++off) {
            if (isTsSyncAt(data, off)) {
                return Container::ts;
            }
        }
    }

    // ---- 再判 PS ----
    // pack_header_start_code = 00 00 01 BA。4 字节固定模式，误判率足够低。
    for (std::size_t i = 0; i + 4 <= limit; ++i) {
        if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01 &&
            data[i + 3] == 0xBA) {
            return Container::ps;
        }
    }

    return Container::unknown;
}

// -----------------------------------------------------------------------------
// 枚举名
// -----------------------------------------------------------------------------

const char* toString(Container c) noexcept
{
    switch (c) {
    case Container::unknown:
        return "unknown";
    case Container::ps:
        return "ps";
    case Container::ts:
        return "ts";
    }
    return "unknown";
}

const char* toString(Codec c) noexcept
{
    switch (c) {
    case Codec::unknown:
        return "unknown";
    case Codec::mpeg1_video:
        return "mpeg1video";
    case Codec::mpeg2_video:
        return "mpeg2video";
    case Codec::mpeg4_video:
        return "mpeg4";
    case Codec::h264:
        return "h264";
    case Codec::h265:
        return "h265";
    case Codec::h266:
        return "h266";
    case Codec::svac_video:
        return "svac";
    case Codec::avs3:
        return "avs3";
    case Codec::aac:
        return "aac";
    case Codec::mp3:
        return "mp3";
    case Codec::ac3:
        return "ac3";
    case Codec::eac3:
        return "eac3";
    case Codec::g711a:
        return "g711a";
    case Codec::g711u:
        return "g711u";
    case Codec::g722:
        return "g722";
    case Codec::g723:
        return "g723";
    case Codec::g729:
        return "g729";
    case Codec::opus:
        return "opus";
    case Codec::svac_audio:
        return "svac-audio";
    }
    return "unknown";
}

} // namespace mpegmech
