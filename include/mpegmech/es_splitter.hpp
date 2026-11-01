// =============================================================================
//  mpegmech - ES 切帧器
// =============================================================================
//  文件    : mpegmech/es_splitter.hpp
//  用途    : 把**任意切分**的 H.264/H.265 AnnexB 字节流切成完整帧。
//
//  ★ 为什么单独一个类，不塞进 Muxer
//  ---------------------------------------------------------------------------
//  切帧和封装是两件事：
//
//      裸流任意切片 → [EsSplitter] → 完整帧 → [Muxer::write] → PS/TS
//                       （只管边界）      （只管封装）
//
//  塞进 Muxer 的话，"时间戳怎么给"会变糊：一次 feed 可能跨好几帧，
//  调用方没法给每帧一个 pts。分开之后，切出来的每一帧都过一次回调，
//  调用方在那里按自己的节奏推进时间戳——真实采集场景本来就知道每帧
//  是什么时候拿到的。
//
//  ★ 输入可以是什么
//  ---------------------------------------------------------------------------
//  任意字节，不需要对齐，不需要落在帧边界，甚至可以是半个起始码：
//
//      splitter.feed(buf1, on_frame);   // 里面可能只有半帧
//      splitter.feed(buf2, on_frame);   // 接着来，跨块的起始码也能认出来
//
//  内部只保留最多 3 字节尾巴（`00 00 01` 可能被切在两次 feed 之间）。
//
//  ★ 怎么判断一帧结束
//  ---------------------------------------------------------------------------
//  两种策略，看流里有没有 AUD：
//
//    1. **有 AUD**（H.264 type 9 / H.265 type 35）：遇到下一个 AUD 就是新帧。
//       最可靠，ffmpeg 编码默认带。
//
//    2. **没有 AUD**：遇到下一个 VCL NAL（H.264 type 1-5 / H.265 0-31 的
//       slice）且 `first_mb_in_slice == 0` 就是新帧。
//       这需要解析 slice header 的第一个 ue(v)，比按 AUD 复杂但可靠。
//
//  ★ "非开头输入"（很重要的语义）
//  ---------------------------------------------------------------------------
//  如果喂进来的第一块**不是帧边界**（比如从 TCP 流中间接入），本类会：
//
//    丢到第一个起始码为止 → 从那里开始当第一帧 → 计入 `dropped_leading_bytes`
//
//  这是唯一安全的做法：残片缺 slice header，解不出画面。
//
//  @warning 但"从第一帧开始"**不代表能立刻解码** —— 如果第一帧不是 IDR
//           （关键帧），解码器会因为缺参考帧而黑屏，直到下一个 IDR 到来。
//           要保证首帧可解，只能由调用方从 IDR 开始喂。
//
//  @note 不做的事：不判断哪个是 IDR（那是 Muxer 的 FrameFlag 该管的），
//        不解析 SPS 里的分辨率（那是 `sps.hpp`）。
// =============================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <system_error>

#include <mpegmech/span.hpp>
#include <mpegmech/types.hpp>

namespace mpegmech {

/// ES 切帧器。
///
/// @par 生命周期
///  切出来的 `frame` 指向**内部分配的缓冲**，回调返回后失效。
///  要保留请自行拷贝（这里刻意不给 shared_ptr：正常用法是拿到就 `write()`）。
///
/// @par 线程
///  非线程安全。一个实例只在一个线程用。
class EsSplitter {
public:
    /// 每切出一帧回调一次。
    using FrameCallback = std::function<void(span<const std::uint8_t> frame)>;

    struct Config {
        /// 视频编码，决定怎么扫 NAL 头。
        Codec codec = Codec::h264;

        /// 单帧上限。超过就丢弃并计 `oversize_drops`。
        ///
        /// @note 1080p 的 IDR 实测 < 1 MiB；4K 可调到 4 MiB。
        std::size_t max_frame_bytes = 1024 * 1024;

        /// 累计缓冲上限。切帧本身不需要缓冲整帧（见实现说明），
        /// 这个值只用于防御"一直喂不到起始码"的畸形输入。
        std::size_t max_buffered_bytes = 4 * 1024 * 1024;

        /// "非开头输入"的处理：第一个起始码之前的残片怎么办。
        ///
        /// - `true`（默认）：丢弃。从第一个 `00 00 01` 才开始当第一帧。
        ///   这是唯一安全的选择 —— 残片缺少 slice header，解不出画面。
        ///
        /// - `false`：把残片当成一帧的开头交出去。**只在明确知道上游已经
        ///   对齐到帧边界时**才这么用；否则会产出坏帧。
        ///
        /// 丢弃了多少字节计入 `stats().dropped_leading_bytes`。
        bool drop_leading_partial = true;
    };

    /// 创建。
    [[nodiscard]] static std::unique_ptr<EsSplitter>
    create(const Config& cfg, std::error_code& ec);

    ~EsSplitter();

    EsSplitter(EsSplitter&&) noexcept;
    EsSplitter& operator=(EsSplitter&&) noexcept;
    EsSplitter(const EsSplitter&)            = delete;
    EsSplitter& operator=(const EsSplitter&) = delete;

    /// 喂入任意切分的字节。
    ///
    /// 每识别出一个完整帧就同步回调一次（可能一次 feed 回调多次）。
    ///
    /// @return 空表示正常；`buffer_overflow` 表示畸形输入已被丢弃重置
    ///
    /// @note **回调里不得调用本对象的 feed/flush**（重入会破坏状态）。
    std::error_code feed(span<const std::uint8_t> data,
                         const FrameCallback& on_frame);

    /// 冲刷残留：把最后一帧交出去。
    ///
    /// @note 流结束时调一次，否则最后一帧会丢。
    std::error_code flush(const FrameCallback& on_frame);

    /// 重置内部状态（换流时用）。
    void reset() noexcept;

    struct Stats {
        std::uint64_t frames_out      = 0;  ///< 切出的帧数
        std::uint64_t bytes_in        = 0;  ///< 累计喂入
        std::uint64_t oversize_drops  = 0;  ///< 超 max_frame_bytes 丢弃的帧数
        std::uint64_t overflow_drops  = 0;  ///< 缓冲溢出重置次数
        std::uint64_t no_startcode    = 0;  ///< 找不到起始码的字节数（诊断）
        std::uint64_t dropped_leading_bytes = 0;  ///< 非开头输入时丢弃的残片字节数
    };
    [[nodiscard]] const Stats& stats() const noexcept;

private:
    struct Impl;
    explicit EsSplitter(std::unique_ptr<Impl> impl) noexcept;

    std::unique_ptr<Impl> impl_;
};

} // namespace mpegmech
