# RTC data-only 能力日志误报复盘

## 现象

THOR 使用 `libdatachannel` 且 `media.enabled=false` 启动时，RTC 日志宣称 peer、media、data 能力均被禁用，无法建立真实 RTC session。

## 影响

实际运行时 PeerConnection 和 DataChannel 均可用，只有媒体 track 被关闭。旧日志会让联调人员误判 ARM64 backend 未编译成功，掩盖真正的视频配置缺口。

## 根因

启动日志把三个独立 capability 用一个 `or` 条件合并，并使用了覆盖所有能力的固定文案，没有区分 data-only 合法模式和 transport 不可用模式。

## 修复

- PeerConnection 或 DataChannel 缺失时才报告无法建立真实 RTC session。
- 仅媒体能力关闭时明确报告 data-only 模式和 video viewer 禁用。

## 规则改进建议

- capability 日志必须逐项描述真实状态，不能把部分降级写成整体不可用。
- 新增合法运行模式时，测试应覆盖对应诊断文本或结构化 diagnostics 值。
