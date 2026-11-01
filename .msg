feat: soak 稳定性工具 + 示例构建段

补上 M6 的最后一项：长时间稳定性可观测。

example/soak.cpp
  同一份 TS 无限循环，每 10 秒打一行 RSS，两种负载分开测（泄漏点不同）：
    reuse  一个 Demuxer 长期存活，喂完 flush + reset 再来
           → 看缓冲/状态机/流表有没有没清干净的累计增长
    churn  每轮 create → feed → destroy
           → 看构造析构路径上的泄漏
  RSS 在 Linux 读 /proc/self/statm，Windows 走 GetProcessMemoryInfo。
  进度打印放在主循环里，不起监视线程 —— 早期版本用线程读计数器是 data race。

CMake
  MPEGMECH_BUILD_EXAMPLES 现在真的会扫 example/*.cpp（之前只是个空开关）
  MSVC 加 _CRT_SECURE_NO_WARNINGS：测试和示例大量用 fopen/getenv，
  换 fopen_s 只会让代码更啰嗦，跨平台也更麻烦

实测（Linux，25 秒 × 2 轮）：
  reuse  823763 轮 / 6178 万帧 / 77.9 GB 输入 / 2599 MB/s / 每轮 0.036 ms
  churn  806431 轮 / 6048 万帧 / 76.3 GB 输入 / 2545 MB/s / 每轮 0.037 ms
  RSS    4.5 MB → 4.6 MB，全程平在 4.6 MB

实测（MSVC，12 秒）：
  churn  393256 轮 / 1861 MB/s / 每轮 0.051 ms / RSS 9.5→10.8 MB
  （Windows 工作集口径本来就抖，两分钟看趋势）

MSVC 侧一并验证：库 9 个源文件 + 3 个测试 + soak，
/W4 /permissive- /utf-8 下零告警；切分不变性测试在 Windows 上结果
与 Linux 完全一致（75/75/163/74）。

README 补了构建、测试素材查找顺序、soak 用法（强调看曲线形状不看斜率）。
