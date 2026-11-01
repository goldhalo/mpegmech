# mpegmech

> **PS / TS 容器**的封装与解封装（C++17，零外部依赖）。
> 核心差异点：**输入容许任意切分** —— 喂进来多少都行，不需要 188 字节对齐、
> 不需要落在 PES 边界、也不需要是完整的一帧。

---

## 为什么再写一个

现有实现的入口契约都把"对齐"推给了调用方：

| | 问题 |
|---|---|
| libmpeg `ts_demuxer_input` | `assert(188 == bytes); assert(0x47 == data[0]);` —— Release 下 `assert` 失效，起点错一位后按 188 硬切，**PID 全乱、不报错、不崩溃，只是解析出垃圾** |
| libmpeg `ps_demuxer_input` | 返回"消费了多少字节"，调用方必须自己 `memcpy` 保存尾巴；漏存/存错就永久错位，同样**没有任何报错** |
| ffmpeg | 能容忍，但体积与依赖不适合嵌入式板卡 |

mpegmech 把这三件事收进库内：**内部暂存 + 同步状态机 + 失步恢复**。
`feed()` 承诺全部消费，返回值只报错，不报"消费了多少"。

---

## 快速开始

### 解封装：容器流 → ES 帧

```cpp
#include <mpegmech/mpegmech.hpp>

using namespace mpegmech;

DemuxerConfig cfg;                          // 默认：自动嗅探 PS / TS
std::error_code ec;

auto dx = Demuxer::create(
    cfg,
    [](const FrameInfo& fi, span<const std::uint8_t> es) {
        // fi.codec / fi.pts / fi.dts / fi.flags 已经填好
        // es 是 AnnexB（H.26x）或 ADTS（AAC），只在本次回调内有效
        onFrame(fi, es);
    },
    [](const StreamInfo& si) {
        // 新流出现 / codec 变化，保证在该流第一帧之前触发
    },
    ec);

if (!dx) { /* ec 说明原因 */ }

// 网络接收：块长度随意，不需要对齐
while (auto n = sock.recv(buf, sizeof(buf))) {
    dx->feed(span<const std::uint8_t>(buf, n));
}
dx->flush();                                 // 流结束
```

### 封装：ES 帧 → 容器流

```cpp
MuxerConfig cfg;                             // 默认 TS
std::error_code ec;

auto mx = Muxer::create(cfg,
    [](span<const std::uint8_t> out) {
        // out 可直接发送：TS 是 188 的整数倍，PS 是完整 pack
        sock.send(out.data(), out.size());
    },
    ec);

StreamConfig sc;
sc.codec = Codec::h264;
int sid = -1;
mx->addStream(sc, sid);

mx->write(sid, FrameFlag::key, pts, dts, annexb_frame);   // 一帧一次
mx->flush();
```

---

## 支持范围

| 项 | 内容 |
|---|---|
| 容器 | MPEG-2 PS（含 MPEG-1 兼容流读取）、MPEG-2 TS（单节目封装，多节目读取） |
| 视频 | H.264 / H.265 / H.266 / MPEG-4 / MPEG-2 / MPEG-1 / SVAC / AVS3 |
| 音频 | AAC / MP3 / AC3 / EAC3 / G.711A / G.711U / G.722 / G.723 / G.729 / OPUS / SVAC |
| v1 优先实现 | H.264 + H.265 + AAC + G.711A/U |
| 依赖 | 仅 C++17 标准库 |
| 平台 | Windows x64（开发）、Linux aarch64（板端） |

**不做**：解码/编码、RTP 打包、RTSP 信令、转码、裸 ES 拆帧（mux 要求整帧输入）。

---

## 关键契约

| 契约 | 说明 |
|---|---|
| `feed()` 全部消费 | 不返回消费字节数；未成帧的尾巴由库内部暂存 |
| 回调同步执行 | 在 `feed()` / `flush()` 栈内触发，串行不并发；回调里不得重入 |
| `payload` 生命周期 | 指向库内缓冲，**回调返回后失效** |
| 单帧一次 memcpy | 一帧在 TS 里散布在多个 188 包中，物理上不连续，零拷贝不成立 |
| 线程模型 | 单对象非线程安全；不同对象无共享状态 |
| 错误传递 | `std::error_code`，不用异常 |

---

## 构建

```bash
cmake -S . -B build -G "Visual Studio 16 2019" -A x64
cmake --build build --config Release
./build/bin/mpegmech_ut
```

交叉编译（aarch64）见 `CMakeLists.txt` 头部注释。

---

## 文档

- **概要设计**：[`doc/概要设计-20261031.md`](doc/概要设计-20261031.md)
  —— 定位、架构、同步算法、内存模型、测试策略、风险与里程碑

---

## 许可证

MIT。参考实现的版权说明见 `NOTICE`（本库按 ISO/IEC 13818-1 标准实现，
不复制第三方代码；若后续借用，会逐文件在此声明）。
