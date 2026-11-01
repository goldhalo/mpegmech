feat: TS 封装 + 解封装（H264/H265 + AAC，支持非整包输入）

TS 是表驱动的：188 字节包本身不带"我是什么"，PID 0 的 PAT 指出 PMT
在哪，PMT 再指出哪个 PID 是视频/音频。所以 TS 不能像 PS 那样靠起始码
自同步，从流中间接入必须等一个 PSI 周期。

  ts_muxer.hpp/cpp
    - crc32_mpeg2 / build_pat / build_pmt / write_ts_packets
    - 首帧前必发 PAT+PMT，之后按 psi_period_ms 周期重发

  Muxer 的 TS 分支
    - PID 自动分配（视频 0x0100 起，音频 0x0110 起）
    - PCR 按 pcr_period_ms 插在 PCR_PID 的包上（默认取第一条视频流）
    - PES 长度 > 0xFFFF 时写 0（不定长），这是 TS 允许的用法

  ts_demuxer.hpp/cpp（新增，与 PS 路径完全分开）
    - 同步：连续 3 个 0x47 且间隔 188
    - 状态机：searching -> waiting_psi -> ready
    - 按 PID 各攒一份 PES，长度可知就收满即出，长度 0 等下一条 PUSI

三个实现时踩到的坑（都已写进设计文档 6.1）：

1) PMT 的 program_info_length 在偏移 10，不是 12。
   从 12 读，读到的其实是第一条 ES 的 stream_type，然后整条 PMT 的
   ES 循环被整个跳过。症状极隐蔽：状态机进了 ready、resyncs 和
   parse_errors 全是 0，但一帧都出不来 —— 统计健康，数据为空。

2) PES 收尾不能只等 PUSI。TS 最后一包会用 adaptation field 填到 188，
   payload 尾部带 stuffing，直接取到包尾会把垃圾塞进 ES。长度可知时
   按 6+PES_packet_length 裁剪。

3) 一条 PES 里常有几十个 ADTS 帧。ffmpeg 打的 TS 里 88 个 AAC 帧挤在
   6 条 PES 里，按 PES 边界整段交出，调用方拿到的就是"一坨 ADTS"。
   现在按 ADTS frame_length 拆成单帧，PTS 按 1024*90000/sample_rate 递增。

test_ts_roundtrip.cpp：
  - H264 -> TS -> 解回：75 帧，ES 逐字节一致，PTS 一致
  - H265 -> TS -> 解回：同上
  - 音视频混合：两路各自 75 / 88 帧，逐字节一致
  - 中间接入（丢掉前 1200 字节）：解出 74 帧，全部与原始帧逐字节相同
  - ffmpeg 打的 TS（PID 分配/描述符/PCR 都跟我们不同）：2 条流、
    75 + 88 帧、resync=0、err=0

交叉验证：
  ffprobe 识别 h264 + aac；
  ffmpeg -f null 完整解码零错误；
  我们封的 TS 拿 ffmpeg 解、ffmpeg 封的 TS 拿我们解，双向都通。
