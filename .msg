chore: MSVC /W4 告警清零 + 测试素材目录可配置

MSVC 编译验证（cl 19.44.35207，/W4 /permissive- /utf-8 /MD /O2）：
库的 9 个源文件零告警，测试程序链接通过并在 Windows 上跑通完整往返。

修掉 4 个告警：
  - C4099 × 3：Impl 在头文件里是 `struct`，cpp 里定义成 `class`。
    统一成 `struct`（本来就是全 public 的内部实现，用 class 反而误导）。
  - C4244 × 1：ps_muxer 里 `0xC0 | (bs_buffer_bound >> 8)` 是 uint32_t
    表达式，隐式窄化成 uint8_t 会丢数据。补 static_cast。

CMakeLists 加 `/utf-8`：
  源码是 UTF-8 含中文注释。cl 默认按 GBK 读，UTF-8 里某些汉字的末字节
  正好是 0x5C（反斜杠），会被当成续行符把下一行吞掉，报一堆看不懂的
  语法错误。这个坑 GCC 不暴露，只在 MSVC 上出现。

test_ts_roundtrip 的素材查找改成 $MPEGMECH_TEST_DATA → /tmp → 当前目录：
  原来硬编码 /tmp，Windows 上四段全部"跳过"，跑了个寂寞。
  现在可以把素材拷到任意目录、设一个环境变量就在 Windows 上验证。
  产物（out_av.ts）也写到素材同目录。

实测结果（MSVC，素材放 D:\temp\mpegmech-data）：
  H264 75/75、H265 75/75、AV 75+88、中间接入 74/74 逐字节一致、
  ffmpeg 打的 TS 75+88 帧 resync=0 err=0 —— 与 Linux/gcc 结果完全一致
  （同一份 TS 字节数也一致：99264 / 81592 / 147580）。
