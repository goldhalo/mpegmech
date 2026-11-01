// =============================================================================
//  src/error.cpp —— 错误码实现
// =============================================================================
//  文件    : src/error.cpp
//  用途    : MpegErrc -> std::error_category
//  编码    : UTF-8
// =============================================================================

#include "mpegmech/error.hpp"

namespace mpegmech {
namespace {

/// mpegmech 的错误类别实现。
///
/// 说明：`message()` 只给**简短英文**，不拼接上下文（哪个流、哪个字节偏移）。
/// 上下文由调用方在日志里补 —— 库不知道调用方的命名习惯，硬拼反而更难查。
class MpegCategory final : public std::error_category {
public:
    [[nodiscard]] const char* name() const noexcept override { return "mpegmech"; }

    [[nodiscard]] std::string message(int ev) const override
    {
        switch (static_cast<MpegErrc>(ev)) {
        case MpegErrc::ok:
            return "success";
        case MpegErrc::invalid_argument:
            return "invalid argument";
        case MpegErrc::not_configured:
            return "object not configured (create/addStream must be called first)";
        case MpegErrc::unsupported_container:
            return "unsupported container (neither PS nor TS could be detected)";
        case MpegErrc::unsupported_codec:
            return "unsupported codec for this container";
        case MpegErrc::malformed:
            return "malformed container structure";
        case MpegErrc::sync_lost:
            return "lost synchronization beyond the resync budget";
        case MpegErrc::buffer_overflow:
            return "internal buffer overflow (oldest data dropped)";
        case MpegErrc::stream_not_found:
            return "stream id not registered";
        case MpegErrc::waiting_psi:      return "waiting for PAT/PMT";
        case MpegErrc::already_flushed:  return "already flushed";
        case MpegErrc::out_of_memory:    return "out of memory";
        case MpegErrc::internal_error:   return "internal error";
        case MpegErrc::capacity_exceeded:
            return "stream capacity exceeded";
        }
        return "unknown mpegmech error";
    }
};

} // namespace

const std::error_category& mpeg_category() noexcept
{
    // C++11 起的 magic static：初始化线程安全，无需额外加锁。
    static const MpegCategory instance;
    return instance;
}

} // namespace mpegmech
