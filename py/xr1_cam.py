#!/usr/bin/env python3
"""外部摄像头录制器的机器人侧驱动（对端是 Mac 上的 XR1Rec.app）。

mac/install_mac_recorder.sh 的注释里点名"由 python3 py/xr1_cam.py install
推送并调用"，但这个文件之前不存在 —— 这里补上。

为什么不直接在每条命令里跑 ssh：
    每次抓取动作前后都要写一次控制文件，冷启动 ssh 握手要 ~300ms，会把"先开录再
    动"变成"动完才开录"。所以统一走 ControlMaster 复用连接，写一次 ~20ms。
    这也是 mac/xr1rec.swift 选择文件控制而非 socket 的前提（Mac 只开了 22 端口）。

为什么要等 state.json 变成 recording 而不是 touch 完就返回：
    output.startRecording() 是异步的，delegate 回调里才真正开始落帧。touch 完立刻
    动手臂，前几十帧会丢 —— 而"动作起始瞬间"恰恰是回放时最需要的一段。

授权（TCC）：
    相机权限授给 .app bundle，且弹窗只能在 Mac 的 GUI 会话里应答。若 state 是
    awaiting_permission，任何远程操作都无法推进，只能有人去 Mac 上点 Allow。
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time

HOST = os.environ.get("XR1REC_HOST", "192.168.123.138")
USER = os.environ.get("XR1REC_USER", "apple")
KEY = os.environ.get("XR1REC_KEY", os.path.expanduser("~/.ssh/id_xr1rec"))
ROOT = os.environ.get("XR1REC_ROOT", "/Users/apple/xr1rec")
DEVICE = os.environ.get("XR1REC_DEVICE", "FHD C3")
FPS = os.environ.get("XR1REC_FPS", "30")
PRESET = os.environ.get("XR1REC_PRESET", "1080")

HERE = os.path.dirname(os.path.abspath(__file__))
MAC_SRC = os.path.join(os.path.dirname(HERE), "mac")

# ControlMaster：第一条命令建连并常驻 10 分钟，后续复用。
_CM = ["-o", "ControlMaster=auto",
       "-o", f"ControlPath=/tmp/xr1rec-cm-{USER}@{HOST}",
       "-o", "ControlPersist=600"]
_SSH_BASE = ["ssh", "-i", KEY, "-o", "BatchMode=yes",
             "-o", "StrictHostKeyChecking=no",
             "-o", "ConnectTimeout=8"] + _CM


class RecorderError(RuntimeError):
    pass


def sh(cmd: str, timeout: float = 30.0, check: bool = True):
    """在 Mac 上跑一条 shell 命令。"""
    p = subprocess.run(_SSH_BASE + [f"{USER}@{HOST}", cmd],
                       capture_output=True, text=True, timeout=timeout)
    if check and p.returncode != 0:
        raise RecorderError(f"ssh rc={p.returncode}: {p.stderr.strip() or p.stdout.strip()}")
    return p.stdout


def state(timeout: float = 15.0) -> dict:
    """读 Mac 上的 state.json。返回 {} 表示 daemon 还没发布过状态。"""
    out = sh(f"cat {shlex.quote(ROOT)}/state.json 2>/dev/null || true", timeout, check=False)
    out = out.strip()
    if not out:
        return {}
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        return {}          # 正在被原子替换，下一轮再读


def alive(st: dict, max_age: float = 5.0) -> bool:
    """心跳是否新鲜。daemon 以 ~2Hz publish，超过 5s 没动就是死了/卡了。

    注意用 Mac 的时钟没意义 —— 这里比较的是本机 time.time() 和 Mac 写入的 epoch，
    两台机器时钟若差得多会误判，所以 doctor 里单独报一次时钟差。
    """
    hb = st.get("hb")
    return isinstance(hb, (int, float)) and (time.time() - hb) < max_age


def require_ready() -> dict:
    """确认 daemon 在跑、有相机权限、当前空闲。不满足就抛出可读的原因。"""
    st = state()
    if not st:
        raise RecorderError(
            "Mac 上没有 state.json —— daemon 没起来。先跑: xr1_cam.py install")
    if st.get("state") == "awaiting_permission" or st.get("auth") == "notDetermined":
        raise RecorderError(
            "Mac 正在等相机授权弹窗被点 Allow（TCC 只能在 Mac 的 GUI 会话里应答，"
            "远程无法代点）。请到 Mac 屏幕上点 Allow，然后重试。")
    if st.get("auth") in ("denied", "restricted"):
        raise RecorderError(
            f"Mac 相机授权被拒 (auth={st['auth']})。到 系统设置 > 隐私与安全性 > "
            "摄像头 里给 XR1Rec 打开；若列表里没有它，删掉 ~/xr1rec/XR1Rec.app 重新 install。")
    if not alive(st):
        age = time.time() - st.get("hb", 0)
        raise RecorderError(f"daemon 心跳已停 {age:.0f}s（state={st.get('state')}）。重跑 install。")
    if not st.get("session_running", True):
        raise RecorderError(f"AVCaptureSession 没在跑: {st.get('error') or '未知'}")
    return st


# ── 动作 ─────────────────────────────────────────────────────────────────────

def do_install(nobuild: bool = False) -> int:
    """把 Swift 源推到 Mac 并编译/签名/在 GUI 会话里启动。

    编译是幂等的，但**重新编译会让 TCC 授权失效**（授权绑到 cdhash），所以别把它
    放进每次实验的流程里 —— 只在源码改了或 daemon 起不来时跑。

    nobuild=True（`relaunch`）只重启，不编译：daemon 挂了但授权还在时用这个，
    否则一次"顺手 install"就把已经点过的 Allow 作废了，还得再找人去点一次。
    """
    for f in ("xr1rec.swift", "Info.plist", "install_mac_recorder.sh"):
        src = os.path.join(MAC_SRC, f)
        if not os.path.exists(src):
            print(f"缺少 {src}", file=sys.stderr)
            return 2
    sh(f"mkdir -p {shlex.quote(ROOT)}/src")
    scp = ["scp", "-i", KEY, "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no"] + \
          [os.path.join(MAC_SRC, f) for f in ("xr1rec.swift", "Info.plist", "install_mac_recorder.sh")] + \
          [f"{USER}@{HOST}:{ROOT}/src/"]
    p = subprocess.run(scp, capture_output=True, text=True, timeout=120)
    if p.returncode != 0:
        print("scp 失败: " + p.stderr.strip(), file=sys.stderr)
        return 3
    env = (f"XR1REC_ROOT={shlex.quote(ROOT)} XR1REC_DEVICE={shlex.quote(DEVICE)} "
           f"XR1REC_FPS={FPS} XR1REC_PRESET={PRESET} "
           f"XR1REC_NOBUILD={'1' if nobuild else '0'}")
    out = sh(f"{env} bash {shlex.quote(ROOT)}/src/install_mac_recorder.sh 2>&1",
             timeout=300, check=False)
    print(out)
    st = state()
    if st.get("state") == "awaiting_permission" or st.get("auth") == "notDetermined":
        print("\n>>> 需要你去 Mac 屏幕上点一次 Allow（相机授权弹窗）。"
              "点完跑 xr1_cam.py status 确认变成 idle。")
    return 0


def do_devices() -> int:
    print(sh(f"{shlex.quote(ROOT)}/XR1Rec.app/Contents/MacOS/xr1rec devices 2>&1",
             timeout=60, check=False))
    return 0


def do_status() -> int:
    st = state()
    if not st:
        print("state=<无>  (daemon 没起来)")
        return 1
    age = time.time() - st.get("hb", 0)
    print(json.dumps(st, ensure_ascii=False, indent=2, sort_keys=True))
    print(f"\n心跳 {age:+.1f}s 前 -> {'活' if alive(st) else '死/卡'}")
    return 0 if alive(st) else 1


def start(clip: str, wait: float = 8.0) -> dict:
    """开录，阻塞到 daemon 确认真的在录（见模块 docstring）。"""
    require_ready()
    safe = clip.replace("/", "_").replace(" ", "_")
    sh(f"printf %s {shlex.quote(safe)} > {shlex.quote(ROOT)}/ctl/start")
    t0 = time.time()
    while time.time() - t0 < wait:
        st = state()
        if st.get("state") == "recording" and st.get("clip") == safe:
            return st
        if st.get("error"):
            raise RecorderError("开录失败: " + str(st["error"]))
        time.sleep(0.2)
    raise RecorderError(f"{wait:.0f}s 内没等到 state=recording（当前 {state().get('state')}）")


def stop(wait: float = 15.0) -> dict:
    """停录，阻塞到文件 finalise 完成，返回 last（含 path/bytes/duration）。"""
    sh(f"touch {shlex.quote(ROOT)}/ctl/stop")
    t0 = time.time()
    while time.time() - t0 < wait:
        st = state()
        if st.get("state") == "idle" and st.get("last"):
            return st["last"]
        time.sleep(0.2)
    raise RecorderError(f"{wait:.0f}s 内没等到落盘完成（当前 {state().get('state')}）")


def pull(clip: str, dest_dir: str) -> str | None:
    """把 Mac 上的 .mov 取回本机。返回本地路径，取不到返回 None。"""
    os.makedirs(dest_dir, exist_ok=True)
    safe = clip.replace("/", "_").replace(" ", "_")
    remote = f"{ROOT}/clips/{safe}.mov"
    local = os.path.join(dest_dir, f"{safe}.mov")
    # 不设 wall-clock timeout：传输时长跟文件大小成正比，而这条链路 ~1MB/s。
    # 2026-08-18 实测 experiment_17_full.mov 915MB 需要 ~15min，旧的 timeout=600
    # 会在 578MB 处抛 TimeoutExpired，留下一个**字节前缀**的 .mov —— 能播、时长短，
    # 看起来像"录短了"而不是"没取完"。挂死改由 ssh keepalive 兜底。
    p = subprocess.run(["scp", "-i", KEY, "-o", "BatchMode=yes",
                        "-o", "StrictHostKeyChecking=no",
                        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=8",
                        f"{USER}@{HOST}:{remote}", local],
                       capture_output=True, text=True)
    if p.returncode != 0:
        print(f"取回 {clip} 失败: {p.stderr.strip()}", file=sys.stderr)
        return None
    return local


def do_doctor() -> int:
    """一次把所有会挡住录制的东西查完，别一个个试。"""
    ok = True
    print(f"目标: {USER}@{HOST}:{ROOT}  设备='{DEVICE}' {PRESET}p{FPS}")
    try:
        t_local = time.time()
        # macOS 的 date 是 BSD 版，没有 %N（会原样输出 "N"）。只取整秒。
        remote_epoch = float(sh("date +%s", timeout=15).strip())
        print(f"[ok] ssh 免密 (密钥 {KEY})；两机时钟差 {remote_epoch - t_local:+.0f}s")
    except Exception as e:
        print(f"[FAIL] ssh: {e}")
        return 1
    st = state()
    if not st:
        print("[FAIL] 没有 state.json —— 跑 install")
        return 1
    print(f"[{'ok' if alive(st) else 'FAIL'}] 心跳 {time.time() - st.get('hb', 0):+.1f}s")
    print(f"[{'ok' if st.get('auth') == 'authorized' else 'FAIL'}] 相机授权 auth={st.get('auth')}")
    if st.get("auth") != "authorized":
        print("      -> 去 Mac 屏幕点 Allow（远程代点不了）")
        ok = False
    print(f"[{'ok' if st.get('session_running') else 'FAIL'}] session_running={st.get('session_running')}")
    print(f"[--] state={st.get('state')} device={st.get('device')}")
    if st.get("error"):
        print(f"[warn] 上次错误: {st['error']}")
    return 0 if ok and alive(st) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="XR1 外部摄像头录制器（Mac 侧）驱动")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("install", help="推源码 + 编译 + 启动 daemon（会重置 TCC 授权）")
    sub.add_parser("relaunch", help="只重启 daemon，不编译（保住已有的相机授权）")
    sub.add_parser("devices", help="列 Mac 上可见的相机")
    sub.add_parser("status", help="打印 state.json")
    sub.add_parser("doctor", help="把所有会挡住录制的原因一次查完")
    s = sub.add_parser("start", help="开录"); s.add_argument("clip")
    sub.add_parser("stop", help="停录")
    sub.add_parser("quit", help="让 daemon 退出")
    p = sub.add_parser("pull", help="取回 clip")
    p.add_argument("clip"); p.add_argument("--dest", default=".")
    t = sub.add_parser("selftest", help="录一小段再取回，验证整条链路")
    t.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()

    try:
        if a.cmd == "install":
            return do_install()
        if a.cmd == "relaunch":
            return do_install(nobuild=True)
        if a.cmd == "devices":
            return do_devices()
        if a.cmd == "status":
            return do_status()
        if a.cmd == "doctor":
            return do_doctor()
        if a.cmd == "start":
            print(json.dumps(start(a.clip), ensure_ascii=False)); return 0
        if a.cmd == "stop":
            print(json.dumps(stop(), ensure_ascii=False)); return 0
        if a.cmd == "quit":
            sh(f"touch {shlex.quote(ROOT)}/ctl/quit"); print("已请求退出"); return 0
        if a.cmd == "pull":
            lp = pull(a.clip, a.dest)
            print(lp or "(失败)"); return 0 if lp else 1
        if a.cmd == "selftest":
            name = f"selftest_{int(time.time())}"
            t0 = time.time(); start(name)
            print(f"开录耗时 {time.time() - t0:.2f}s")
            time.sleep(a.secs)
            last = stop()
            print(f"落盘: {last.get('bytes')} B, {last.get('duration'):.2f}s")
            lp = pull(name, "/tmp")
            print(f"取回: {lp} ({os.path.getsize(lp) if lp else 0} B)")
            return 0 if lp and os.path.getsize(lp) > 10000 else 1
    except RecorderError as e:
        print(f"错误: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
