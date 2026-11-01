fix: PSM 的 elementary_stream_map_length 写成了条数，应该是字节数

加 G.711 支持时才发现这个字段一直是错的 —— 之前两轮测试都没暴露。

  PSM 结构
    [current_next+version 2B][program_stream_info_length 2B][info]
    [elementary_stream_map_length 2B][每条 4B: stream_type + sid + info_len]
    [CRC32 4B]

  elementary_stream_map_length 是**映射区的字节数**（每条 4 字节），
  我按"流的条数"写（1 条 → 0x0001）。解封装侧判断 `remaining >= 4`，
  1 永远不成立，整张 PSM 被当成空表跳过。

为什么两轮测试都没炸：
  视频靠 stream_id 0xE0 就能猜出 H.264，AAC 靠 ADTS 同步字也猜得出，
  所以 ffprobe 一直报 h264 + aac —— 看起来是对的。
  只有 G.711 这种**只能靠 PSM 认**的流才会露馅：codec 解出来是 unknown。

教训：验证一张表，要用"只能靠这张表才能得到"的用例。
用"猜也猜得对"的流去测，等于没测。

test_g711_ps.cpp
  - 造 8 kHz / 20 ms 的 G.711 裸流（锯齿波，错位能看出来）
  - PS 封装 → 37 字节不对齐地喂回去 → 帧数、codec、码流、PTS 间隔全比
  - A-law / μ-law 各跑一遍

交叉验证：
  G.711A → ffprobe 认成 pcm_alaw / 8000 Hz / 单声道，ffmpeg 解码零错误
  G.711U → ffprobe 认成 mp2。这是 ffmpeg 的 PS 侧没做 0x91 映射，
           与码流无关（本库自己两种都能认回来）。对端是 ffmpeg 时用 A-law。
  改完复验 AV PS：仍为 h264 + aac，解码零错误。

设计文档 §6.2 记了这个坑和 G.711 的放法。
