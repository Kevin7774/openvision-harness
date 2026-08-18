# 实验记录 20260817-01「yellow-vista-preflight-20260817」

## 结论

**裁定：`crashed`**（抛异常退出）

- error：AttributeError: 'Namespace' object has no attribute 'detector'

> 裁定一律照抄 `meta.json` 的 `outcome`，本文件**不重新裁定** —— 重判日志是 `scripts/adjudicate.py`（`docs/05 §E1.3`，待写）的活，它有「人工标注 20/20 一致」的退出判据。

## 进度：走到了第几关

**0/5 关**（这一族共 9 关） —— 上限是 5 而不是 9，因为 `dry_run` 开着，设计上只跑到「离地余量通过」

- 到达：**一关都没过**
- 止步于：**感知到积木**（`perceive`）
- ⬜ 没达到本次的设计上限

| 关 | | 状态 |
|---|---|---|
| 1 | 感知到积木 | ⬜ |
| 2 | 按颜色筛选 | ⬜ |
| 3 | 选定目标 | ⬜ |
| 4 | IK 出解 | ⬜ |
| 5 | 离地余量通过 | ⬜ |
| 6 | 到预抓位 | ⬜ |
| 7 | 到抓取高度 | ⬜ |
| 8 | 闭合夹爪 | ⬜ |
| 9 | 起吊并举证 | ⬜ |

> ✅ 到达　⬜ 未到达　➖ 本次参数下没有这一关（比如没给 `--color` 就不会筛色）

> **走得远 ≠ 成功。** 阶梯只说明没在半路报错；成没成看上面的裁定。`20260810-14` 阶梯走满却是 `empty`（夹了个空）就是这个道理（`PITFALLS.md §24`）。

## 相比上一次

这是 `grasp_block` 这一族的**第一次**试验，没有可比对象。

**人话注记**：（未填）—— 在 `experiments/lineage.json` 里给 `20260817-01` 加一条 `{changed, gained, lost}`，重新生成即可。

## 关键测量

这次没有可提取的测量量。

## 条件与素材

### 头部姿态

（这次没有任何事件记下头部关节角）

> `docs/05 §E1.3` 要求「每条 episode 的判定必须在同一个头部姿态下做」：ZED 挂在头上，`head_pitch` 一变外参就变。头必须俯到 +40° 限位，否则相机和手臂的可达区完全不重叠（`docs/10 §5.1`）。

### 外部摄像头素材

- 片段：`20260817-02/clips/` 里的 `20260817-02__01_yellow-vista-preflight-20260817`
- 6 帧 @ 1920x1080
- 先开录 6.61s（`rec_confirm_ms`，纯本机钟）

> 排序证据只看 `rec_confirm_ms`（本机钟）。`lead_s` 减的是 Mac 写的时间戳，把两机钟差也算进去了，差大时会变成负数 —— 那是时钟问题不是「先动后录」（`PITFALLS.md §30`）。

**积木真值位姿：无。** ArUco 真值通道未搭（docs/05 §E1.2，优先级已下调）

> `docs/05 §4.3` 把真值列为「缺一项该次作废」的记录项。这里显式记 null + 原因而不是省掉 —— 省掉会让后来的人以为那次测了。所以本次**不能**用来算感知误差（H3）。

### 代码指纹

| 脚本 | sha1 |
|---|---|
| `base_nudge.py` | `39d5e9dd2e09` |
| `calib_probe.py` | `e6216342c445` |
| `exp_log.py` | `847ce750c795` |
| `global_edge_grasp.py` | `d6bc9256d647` |
| `grasp_block.py` | `9b22f20ad6b0` |
| `grasp_zero_calib.py` | `2168e9903be3` |
| `ik_probe.py` | `5e2f9e65cb76` |
| `perc_repeat.py` | `896e935e7209` |
| `retry_edge_grasp.py` | `1c81cd374224` |
| `safe_retreat.py` | `931b52e1ad6a` |
| `tip_probe.py` | `f22c175984b8` |
| `xr1.py` | `0faaacc4eb57` |

> workspace 不是 git 仓库，所以「这次结果对应哪版代码」退化成记关键脚本的 sha1。改了常量忘了记，就是在给自己制造无法复现的实验。

---

原始事实在 `meta.json`（一次写死）和 `record.jsonl`（6 行事件流）。本文件由 `scripts/exp_report.py` 从那两个文件生成，可以随时重新生成，**不要手改**。
