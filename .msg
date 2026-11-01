feat: TS 多节目（每套节目各一个 PMT，按 program_number 分流）

README 一直写着"多节目读取"，实现上其实只记了 PAT 里**最后一个** PMT PID，
多节目流等于只解最后一套，FrameInfo.program 还恒填 1。现在补上。

  TsDemuxer
    - m_pmt_pid + m_have_pat  →  m_programs（PMT PID → program_number 的整张表）
    - PAT 整张重建（一次 PAT 就是当前完整节目表），已消失的节目连带清流表
    - PMT 只替换**本节目**下的流表项，别的节目不动
    - TsEsInfo / Frame 加 program 字段

  Demuxer
    - StreamInfo.program / FrameInfo.program 用真实 program_number（PS 仍恒为 1）
    - push_ts_frame 多带一个 program 参数

test_ts_multiprogram.cpp
  手工拼一个双节目 TS（我们自己的复用器只做单节目）：

    PAT ─┬─> program 1 → PMT@0x1000 → ES 0x100 (H.264)
         └─> program 2 → PMT@0x1001 → ES 0x200 (AAC)

  两路 ES 交错写、中途重发一次 PSI，用 97 字节不对齐地喂。
  结果：2 条流、program 分别 1 / 2、码流逐字节一致、resync=0 err=0。

  顺带把"PAT/PMT 的字段在哪个偏移"钉死在测试里 —— 上一轮那个
  program_info_length 偏移写错（10 当成 12）就是这么漏掉的：
  单节目流只要有一次对上了就看不出来。

CMake：测试目标加上 src 作为 include 目录（手工拼容器要用内部头）。
MSVC 侧 12 个用例零告警。
