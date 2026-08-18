# CLAUDE.md — zed_ws

动机器人之前先读 [STATUS.md](./STATUS.md)（现状快照）和 [PITFALLS.md](./PITFALLS.md)（踩过的坑）。

## 抓积木一律走这个闭环（操作者 2026-08-14 定，不要再写一次性脚本）

`scripts/agent_loop.py` 是这个循环的驱动，**④推理那一步是我自己，不在代码里**：

```
①第一次观察 ②当前视觉状态 ③当前可执行动作 -> observe / actions
④agent 推理                                -> 我
⑤查历史视觉 ⑥更新记忆                      -> history / remember（落 LOOP_MEMORY.md）
⑦预测结果 ⑧执行一个动作 ⑨环境变化          -> act <名字> --predict "..." [--go]
⑩返回帧 ⑪进视觉记忆 ⑫末帧成为当前状态      -> act 结束时自动 observe
⑬比较预测和现实 ⑭修改理解 ⑮下一轮          -> act 打的 PREDICT/ACTUAL + remember
```

```bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
cd /home/astrabot/workspace/zed_ws/scripts
python3 xr1.py bringup                     # 夹爪驱动不是 systemd 服务
python3 agent_loop.py new --label 抓黄积木
python3 agent_loop.py act look 40 0 --predict "…"   # 头 pitch 必须 +40，yaw 钉死 0
python3 agent_loop.py actions               # 动作空间：look/solve/pregrasp/descend/
                                            # close/open/lift/park/probe
python3 agent_loop.py act solve 0.48 -0.19 --predict "…"     # IK 分钟级，落盘复用
python3 agent_loop.py act pregrasp --predict "…" --go        # 不给 --go 只规划
```

### 自动编排（一次一个动作，默认不动硬件）

`scripts/agent_runner.py` 在上述工具层外增加最小状态机：
`OBSERVE → DECIDE → EXECUTE_ONE → VERIFY → REFLECT`。模型只能返回 JSON 并选择
`agent_loop.py actions` 里的白名单动作；实验、证据和假设写进当前 run 的
`runner_state.json` / `runner_journal.jsonl`，完整历史不回灌模型。

```bash
# 先退出其他会控制机器人的 Claude/agent 会话，并完成上面的 ROS 环境初始化。
python3 agent_runner.py new \
  --goal "抓起黄色积木并放到绿色区域" \
  --success "末帧中黄色积木位于绿色区域，夹爪已张开且未持物"

python3 agent_runner.py step          # 首轮 dry-run：观察并决策，不执行物理动作
python3 agent_runner.py context       # 审查下一轮实际喂给模型的精简上下文
python3 agent_runner.py status
python3 agent_runner.py step --go     # 调试期一次只授权一个物理动作
python3 agent_runner.py run --go --cycles 5  # 稳定后才允许小批量连续闭环
python3 agent_runner.py audit         # 核对实验编号、MP4/Markdown 一一对应和有效帧
python3 agent_runner.py resume        # BLOCKED 且人工检查现场后才恢复
```

`new` 里的目标位置必须写成可由 ZED 验证的具体对象/区域，不能只写“指定位置”。
每个 `step --go` 自动成为一个连续编号的 `Experiment NN`：Mac 在观察前开始连续录像，
动作后观察完成才停；目录内只保留 `experiment_NN_full.mp4` 和 `experiment_NN.md`。
下一轮会把预测对账、错误分析、最小修复和新假设回写到上一份报告。dry-run `step`
不编号、不录像，也不会移动硬件。任务声明 `DONE` 前会自动 `audit`，审计失败转入
`BLOCKED`，不会生成成功的 `FINAL RESULT.md`。

先用 `step` 跑通模型结构化输出；任何模型超时、非法动作、证据不足、预算耗尽或工具失败
都会进入 `BLOCKED`，不会沿用旧决定继续运动。不要并发运行两个 runner，也不要同时手工
调用 `agent_loop.py act`。

四条规矩：**一次一个动作**（做完必须重新观察）；**每个 act 必须带 `--predict`**（不写
就没有 ⑬ 可对账）；**不自己写运动学/安全判据**（一律走 `grasp_block` 的 IK＋碰撞＋
指尖地板检查，放宽任何判据要人授权）；**先开 `frames/*_marked.jpg` 看原图**再读聚合值。

## Agent skills

> 供 mattpocock 系列工程 skill（`/tdd`、`/code-review`、`/research`、`/prototype`、
> `/to-tickets`、`/triage`、`/wayfinder` 等）读取的三份配置，正文在 `docs/agents/`。

### Issue tracker

本目录没有 git remote，issue 以 markdown 文件存放在 `.scratch/<feature>/`。见 [`docs/agents/issue-tracker.md`](./docs/agents/issue-tracker.md)。

### Triage labels

沿用五个默认标签（`needs-triage` / `needs-info` / `ready-for-agent` / `ready-for-human` / `wontfix`），标签串与角色名一致。见 [`docs/agents/triage-labels.md`](./docs/agents/triage-labels.md)。

### Domain docs

single-context：根目录一份 `CONTEXT.md` ＋ `docs/adr/`，两者目前都不存在，按需懒创建；在它们出现之前，领域知识的实际落点是 `STATUS.md`、`PITFALLS.md` 和 memory 目录。见 [`docs/agents/domain.md`](./docs/agents/domain.md)。
