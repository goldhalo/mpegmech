// =============================================================================
//  adts.cpp —— ADTS 头生成与 ASC 解析
// =============================================================================
#include "adts.hpp"

#include <cstddef>
#include <cstdint>

namespace mpegmech {
namespace detail {

namespace {
/// 采样率表（ISO 14496-3 Table 1.18）
constexpr std::uint32_t kSampleRates[] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050,
    16000, 12000, 11025, 8000,  7350
};
constexpr std::size_t kSampleRateCount = sizeof(kSampleRates) / sizeof(kSampleRates[0]);
} // namespace

std::uint32_t sampling_rate_of(std::uint8_t index) noexcept
{
    return (index < kSampleRateCount) ? kSampleRates[index] : 0;
}

AacConfig parse_asc(const std::uint8_t* asc, std::size_t len) noexcept
{
    AacConfig cfg;
    if (asc == nullptr || len < 2) {
        return cfg;   // valid = false
    }

    // 逐位读：5 + 4 + 4 = 13 bit，跨 2 字节
    const std::uint32_t bits = (static_cast<std::uint32_t>(asc[0]) << 8) | asc[1];

    cfg.profile        = static_cast<std::uint8_t>((bits >> 11) & 0x1F);
    cfg.sampling_index = static_cast<std::uint8_t>((bits >> 7) & 0x0F);
    cfg.channel_config = static_cast<std::uint8_t>((bits >> 3) & 0x0F);
    cfg.valid = (cfg.sampling_index < kSampleRateCount) && cfg.channel_config != 0;
    return cfg;
}

void build_adts_header(Bytes& out, const AacConfig& cfg, std::size_t frame_len)
{
    // ADTS 七字节（无 CRC）：
    //
    //  byte0: syncword[11..4]            = 0xFF
    //  byte1: syncword[3..0] | ID | layer | protection_absent
    //  byte2: profile | sampling_idx | private | channel_cfg[2]
    //  byte3: channel_cfg[1..0] | original | home | copyright_id | copyright_start | frame_len[12..11]
    //  byte4: frame_len[10..3]
    //  byte5: frame_len[2..0] | buffer_fullness[10..6]
    //  byte6: buffer_fullness[5..0] | num_raw_blocks

    const std::uint16_t len = static_cast<std::uint16_t>(
        (frame_len > 0x1FFFu) ? 0x1FFFu : frame_len);

    out.push_back(0xFF);

    // syncword 低 4 位 '1111' + ID=0(MPEG-4) + layer=00 + protection_absent=1
    out.push_back(0xF1);

    // profile（objectType - 1，LC 是 2 → 填 1）+ 采样率索引 + private=0 + 声道高 2 位
    const std::uint8_t profile = static_cast<std::uint8_t>((cfg.profile > 0) ? (cfg.profile - 1) : 0);
    out.push_back(static_cast<std::uint8_t>((profile << 6) |
                                            ((cfg.sampling_index & 0x0F) << 2) |
                                            ((cfg.channel_config >> 2) & 0x01)));

    // 声道低 2 位 + original=0 + home=0 + copyright 两位 + frame_len 高 2 位
    out.push_back(static_cast<std::uint8_t>(((cfg.channel_config & 0x03) << 6) |
                                            ((len >> 11) & 0x03)));

    out.push_back(static_cast<std::uint8_t>((len >> 3) & 0xFF));

    // frame_len 低 3 位 + buffer_fullness 高 5 位（VBR 填 0x7FF）
    out.push_back(static_cast<std::uint8_t>(((len & 0x07) << 5) | 0x1F));

    // buffer_fullness 低 6 位 + num_raw_blocks=0
    out.push_back(0xFC);
}

} // namespace detail
} // namespace mpegmech
