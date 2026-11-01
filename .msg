feat: 音频支持（AAC + ADTS）+ 音视频混合 PS

音频部分此前只有字段声明，没有实现。现在补齐：

  adts.hpp/cpp（120 行）
    - parse_asc()        从 AudioSpecificConfig 解析 profile/采样率/声道
    - build_adts_header() 生成 7 字节 ADTS 头（无 CRC）
    - sampling_rate_of()  采样率索引 → Hz

  Muxer 音频分支
    - addStream 时若 codec==aac && add_adts，解析 ASC
    - write 时补 7 字节 ADTS 头，PES 长度相应 +7
    - 音频不做 IDR 扫描（没有关键帧概念）

又一个"少写一个字节"的 bug：

  PSM 的 program_stream_info_length 是 **16 位**，我只写了 1 个字节。
  后面所有字段错位 1 字节，ffmpeg 把 AAC 的 stream_type(0x0F) 读成
  别的值，认成 mp3 / mp2，还报 "Header missing" —— 看起来像 ADTS 头
  写错了，实际是它根本没读到 PSM 的正确位置。

验证：

  test_av_mux        H264 + AAC → PS
                     ffprobe: video=h264 / audio=aac
                     ffmpeg 解码零错误

  test_av_roundtrip  H264 + AAC → PS → 解回
                     视频 75 帧 / 音频 88 帧，帧数一致
                     视频逐字节一致
                     音频 payload = 7 字节 ADTS + 原始裸帧
                     stream 回调恰好 2 次（两个流各一次）
