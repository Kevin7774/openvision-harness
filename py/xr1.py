#!/usr/bin/env python3
"""XR1 unified control layer -- arms, neck, both grippers, all four cameras.

WHY THIS FILE EXISTS
--------------------
Everything here was previously rediscovered from scratch every session: which
topic the grippers listen on, whether 0 means open or closed, which joint raises
a hand and with which sign, which interpreter can import pyzed, that
/dev/video3 is a metadata node. That rediscovery cost more wall-clock than the
motion itself. This module is the answer written down once, executably.

DESIGN RULE: one process, one DDS participant, whole task inside it.
An agent that issues one `ros2 topic pub` per motion step pays DDS discovery
(~2-3 s) AND an LLM round-trip per step against a robot whose control loop runs
at 200 Hz (5 ms). Put the loop -- including any calibration probes -- in here.

VERIFIED GROUND TRUTH (2026-08-07, on hardware, not from docs)
--------------------------------------------------------------
  ROS_DOMAIN_ID=12 is mandatory. Domain 0 shows an almost-empty graph, no error.
  /joint_states                200 Hz, RADIANS, effort is .nan (no contact sense)
  arm cmd    /astrabot_arm_forward_position_controller/commands
             Float64MultiArray, 14 positions, NO interpolation -- ramp yourself
  neck cmd   /astrabot_neck_forward_controller/commands  [head_pitch, head_yaw]
  gripper    /rm_{side}/rm_driver/teleop_gripper_float  Float64
             0.0 = fully OPEN, 1.0 = fully CLOSED   (NOT astra_arm.gripper())
  gripper fb /qg_robot/gripper_{side}_state  UInt32MultiArray
             [pos_mm, running, temp, error]; 840 mm = open, ~5 mm = closed
  cameras    /dev/f_chest_cam (chest)  /dev/l_arm_cam (left wrist)  -- TWO only
             NEVER write video<N> numbers: they change on every replug. The image
             node is the one with ATTR{index}=="0", NOT "the even one".
             open as MJPG or capture hangs.
             right wrist has NO UVC: it is a DaBai DW2 (libusb, no /dev/video*),
             read it as the ROS topic /camera/depth/image_raw (depth+IR, no RGB)
  ZED        only /home/astrabot/deploy/.venv/bin/python has pyzed (py3.10)

HAZARD: astrabot_mpc / astrabot_mrt / astrabot_arbitration are running and also
publish to the arm command topic. They are idle unless something posts to
/reference/cmd, but if they start streaming they will fight every command sent
from here. astra_arm refuses to move when it sees traffic on the channel -- that
refusal is correct, do not bypass it.

CLI
---
    python3 xr1.py status                  # full state probe (xr1_verify.py)
    python3 xr1.py pose                    # current joint angles
    python3 xr1.py look  40 0              # head pitch/yaw in DEGREES
    python3 xr1.py grip  left close        # or: open / 0.0-1.0
    python3 xr1.py wave  right --rounds 2  # raise + wave + gripper + lower
    python3 xr1.py demo  --rounds 2        # both arms alternating
    python3 xr1.py home                    # all arm joints to 0
    python3 xr1.py snap  --all             # capture every camera
    python3 xr1.py blocks                  # ZED -> table plane -> blocks in base_link

Add --record to ANY motion command to film it with the external camera on the
Mac and write a per-action experiment record (see xr1_experiment.py):
    python3 xr1.py rec setup               # one-time: build/launch the Mac recorder
    python3 xr1.py rec new --label 抓积木   # open a run; later actions join it
    python3 xr1.py wave right --record     # films it, logs it, keeps the clip
    python3 xr1.py rec end                 # assemble movie.mp4 + REPORT.md
Recording starts BEFORE the motion: --record blocks until the camera is
confirmed rolling, and the proven lead time is stored in each step's JSON.

Import instead when you want a program:
    from xr1 import XR1
    with XR1() as r:
        r.look_at(pitch_deg=40)
        r.raise_arm("right", 0.7)
        r.wave("right", n=2)
        r.grip("right", 1.0); r.grip("right", 0.0)
        r.home("right")
"""
from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import time

sys.path.insert(0, "/home/astrabot/tools")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ------------------------------------------------------------------ constants
ROS_DOMAIN_ID = "12"
DEPLOY_PY = "/home/astrabot/deploy/.venv/bin/python"
SCRIPTS = os.path.dirname(os.path.abspath(__file__))

GRIP_CMD = "/rm_{}/rm_driver/teleop_gripper_float"
GRIP_STATE = "/qg_robot/gripper_{}_state"
NECK_CMD = "/astrabot_neck_forward_controller/commands"

GRIP_MAX_MM = 840.0
GRIP_TOL_MM = 30.0

# Empirically determined on hardware; the SIGNS ARE MIRRORED between arms and
# cannot be guessed. left_arm_2 raises on +, right_arm_2 raises on -.
RAISE_JOINT = {"left": ("left_arm_2_joint", +1),
               "right": ("right_arm_2_joint", -1)}
# arm_4 flexes the elbow -- the wave joint. left_arm_4's lower limit is only
# -0.139 rad, so a symmetric +-0.40 wave is CLIPPED on the left. Not a bug.
WAVE_JOINT = {"left": "left_arm_4_joint", "right": "right_arm_4_joint"}

# Only two UVC cameras exist. The right wrist's mono camera was REMOVED to fit
# the DaBai DW2 depth camera and, per the operator (2026-08-11), will not come
# back -- so there is no /dev/r_arm_cam. Do not re-add "rwrist" here: the DW2 is
# a libusb vendor-class device with no /dev/video* node and is only reachable as
# the ROS topic /camera/depth/image_raw.
CAMERAS = {"chest": "/dev/f_chest_cam",
           "lwrist": "/dev/l_arm_cam"}

# 右腕不是 UVC：那颗 DECXIN 为装深度相机拆掉了，现在腕上是 Intel RealSense D455，
# 走 realsense2_camera 的 ROS 话题（内核里它虽然也有 /dev/video10，但裸 UVC 拿不到
# 对齐和后处理，而且和驱动抢设备）。**没有这一条的时候 CAMERAS["rwrist"] 是 KeyError**，
# 于是 grasp_block 的"腕部相机随动"那张证据票对右臂恒为 False —— 而桌上的积木全在
# 右臂一侧，等于三票判定只剩两票。
CAM_TOPICS = {"rwrist": "/right_wrist/d455/color/image_raw"}
D455_DEPTH = "/right_wrist/d455/aligned_depth_to_color/image_raw"

# The ZED is served by zed_wrapper on the ROS graph, NOT by opening pyzed.
ZED_RGB_COMPRESSED = "/zed/zed_node/rgb/color/rect/image/compressed"
ZED_DEPTH = "/zed/zed_node/depth/depth_registered"
ZED_CLOUD = "/zed/zed_node/point_cloud/cloud_registered"
ZED_OPTICAL_FRAME = "zed_left_camera_frame_optical"   # NOT ..._optical_frame

DEG = 57.29577951308232


def _env_guard():
    if os.environ.get("ROS_DOMAIN_ID") != ROS_DOMAIN_ID:
        raise SystemExit(
            f"ROS_DOMAIN_ID is {os.environ.get('ROS_DOMAIN_ID')!r}, must be "
            f"{ROS_DOMAIN_ID}.\n"
            f"  export ROS_DOMAIN_ID={ROS_DOMAIN_ID} "
            f"RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
            "A wrong domain shows an almost-empty graph and NO error message.")


class XR1:
    """One DDS participant; arms, neck, grippers and cameras all on it."""

    def __init__(self, verbose=True):
        _env_guard()
        self.verbose = verbose
        self.t0 = time.time()

        import astra_arm
        import rclpy
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64, Float64MultiArray, UInt32MultiArray
        from tf2_ros import Buffer, TransformListener

        self._rclpy = rclpy
        self._Duration = Duration
        self._F64 = Float64
        self._F64MA = Float64MultiArray
        self.MotionRefused = astra_arm.MotionRefused

        # astra_arm.Robot brings the safety layer we want and already proved
        # itself: ramps from the MEASURED pose, caps per-joint velocity, clamps
        # to live URDF limits, holds uncommanded joints, refuses on stale state
        # or a busy channel.
        self.robot = astra_arm.Robot()
        self.node = self.robot.node
        self._log(f"Robot() ready ({time.time()-self.t0:.2f}s)")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._grip_pub = {s: self.node.create_publisher(
            Float64, GRIP_CMD.format(s), qos) for s in ("left", "right")}
        self._grip_pos: dict[str, int | None] = {"left": None, "right": None}
        for s in ("left", "right"):
            self.node.create_subscription(
                UInt32MultiArray, GRIP_STATE.format(s),
                lambda m, s=s: self._on_grip(s, m), 10)
        self._neck_pub = self.node.create_publisher(Float64MultiArray,
                                                    NECK_CMD, qos)
        # TF is created LAZILY on first tcp_z() call. /tf runs at 200 Hz and a
        # TransformListener on this single-threaded executor starves the
        # /joint_states callback enough to trip astra_arm's 0.3 s staleness
        # guard. Only the verification path needs TF, so don't pay for it.
        self._Buffer, self._TFListener = Buffer, TransformListener
        self.tf = None
        self.spin(1.0)
        self._log(f"grippers + neck attached ({time.time()-self.t0:.2f}s)")

    # --------------------------------------------------------------- plumbing
    def _log(self, msg):
        if self.verbose:
            print(f"[xr1 t+{time.time()-self.t0:6.2f}s] {msg}", flush=True)

    def _on_grip(self, side, msg):
        if len(msg.data) >= 1:
            self._grip_pos[side] = msg.data[0]

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.01)

    def close(self):
        try:
            self.robot.close()
        except Exception:                                     # noqa: BLE001
            pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # ------------------------------------------------------------------ arms
    def joints(self, fresh=True):
        """Measured joints. Retries once on a marginal staleness trip.

        Not a bypass of astra_arm's safety guard: the retry spins longer and
        re-reads genuinely fresh data, then re-applies the same check. A real
        dead /joint_states still raises.
        """
        try:
            return self.robot.joints(fresh=fresh)
        except self.MotionRefused as exc:
            if "stale" not in str(exc):
                raise
            self.spin(0.4)
            return self.robot.joints(fresh=fresh)

    def pose(self, fresh=True):
        """fresh=False 只读最近一次收到的 /joint_states，**不 spin**。

        给后台采样线程用（xr1_experiment.Step 的运动采样）：`fresh=True` 会
        `rclpy.spin_once` 同一个 node，和正在做动作的主线程抢 spin —— 轻则采到的
        点乱序，重则主线程的新鲜度检查扑空、动作中途被 MotionRefused 打断。
        为一个"影片里剪掉静止"的功能去冒这个风险不值得。
        """
        j = self.joints(fresh=fresh)
        out = {}
        for side in ("left", "right"):
            out[side] = [j.get(f"{side}_arm_{i}_joint") for i in range(1, 8)]
        out["neck"] = [j.get("head_pitch_joint"), j.get("head_yaw_joint")]
        return out

    def move(self, targets, speed=0.6, dry_run=False):
        """targets: {joint_name: radians}. Ramped, clamped, velocity-capped."""
        return self.robot.move(targets, speed=speed, dry_run=dry_run)

    def move_through(self, waypoints, speed=0.6, guard=None):
        """One streamed ramp through a joint-space polyline. See astra_arm."""
        return self.robot.move_through(waypoints, speed=speed, guard=guard)

    def raise_arm(self, side, amount=0.70, speed=0.6):
        """Lift the hand. Sign is per-arm and mirrored -- handled here."""
        j, sign = RAISE_JOINT[side]
        q0 = self.joints()[j]
        self.move({j: q0 + sign * amount}, speed=speed)
        self._log(f"{side}: raised {amount:.2f} rad on {j} (sign {sign:+d})")
        return q0

    def lower_arm(self, side, q0, speed=0.6):
        j, _ = RAISE_JOINT[side]
        self.move({j: q0}, speed=speed)
        self._log(f"{side}: lowered")

    def wave(self, side, n=2, amp=0.40, speed=0.8):
        j = WAVE_JOINT[side]
        w0 = self.joints()[j]
        for _ in range(n):
            self.move({j: w0 + amp}, speed=speed)
            self.move({j: w0 - amp}, speed=speed)
        self.move({j: w0}, speed=speed)
        self._log(f"{side}: waved {n}x on {j} (left is clipped by its "
                  f"-0.139 rad lower limit -- expected)")

    def home(self, which="both", speed=0.5):
        return self.robot.home(which)

    def tcp_z(self, side):
        """Hand height in base_link -- the only way to verify a raise happened,
        since effort is .nan and cannot be used.

        NOTE tcp_link is ONE finger, and this drops the rotation. Do not use it
        as a table-collision floor: with a tilted gripper the fingertip midpoint
        sits anywhere from 48.5mm below tcp_link to *above* it. Measured on the
        2026-08-11 known-good grasp pose: tcp_z 0.8417 but tip midpoint 0.8238,
        a 17.8mm gap, not 48.5mm. Use tip_center() for anything safety-related.
        """
        tf = self._lookup(f"{side}_tcp_link")
        return None if tf is None else tf.transform.translation.z

    def _lookup(self, frame, target="base_link"):
        if self.tf is None:                     # lazy: see __init__ for why
            self.tf = self._Buffer()
            self._TFListener(self.tf, self.node)
            self.spin(1.0)
        try:
            return self.tf.lookup_transform(
                target, frame, self._rclpy.time.Time(),
                timeout=self._Duration(seconds=0.4))
        except Exception:                                     # noqa: BLE001
            return None

    def tip_center(self, side, offset):
        """**Measured** fingertip-midpoint in base_link, rotation included.

        `offset` is the tcp->fingertip-midpoint vector in the tcp frame; pass
        `grasp_block.TIP_CENTER` (kept there so the geometry lives in one file).
        This is the quantity every table-clearance check must use -- see the
        warning in tcp_z(). Returns a 3-vector, or None if TF is unavailable.
        """
        import numpy as np
        tf = self._lookup(f"{side}_tcp_link")
        if tf is None:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        n = (x * x + y * y + z * z + w * w) ** 0.5
        x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return np.array([t.x, t.y, t.z]) + R @ np.asarray(offset, dtype=float)

    # ------------------------------------------------------------------ neck
    def look_at(self, pitch_deg=None, yaw_deg=None, steps=40, dwell=0.03):
        """Head pitch/yaw in DEGREES. Hard limit is +-40 deg on both.

        The table is ONLY visible near pitch=+40 (the limit): at pitch 0 the
        table starts at x>0.826 m while the hand cannot reach past x~0.49, so
        camera and reach do not overlap at all.
        """
        j = self.joints()
        p0 = j.get("head_pitch_joint", 0.0)
        y0 = j.get("head_yaw_joint", 0.0)
        lim = self.robot.limits          # {joint: (lower, upper)} from live URDF

        def clamp(name, v, default=0.698):
            lo, hi = lim.get(name, (-default, default))
            return max(lo, min(hi, v))

        p1 = p0 if pitch_deg is None else clamp("head_pitch_joint", pitch_deg / DEG)
        y1 = y0 if yaw_deg is None else clamp("head_yaw_joint", yaw_deg / DEG)
        # The neck controller has no interpolation either -- ramp it.
        for k in range(steps + 1):
            a = k / steps
            self._neck_pub.publish(self._F64MA(
                data=[p0 + (p1 - p0) * a, y0 + (y1 - y0) * a]))
            self.spin(dwell)
        self.spin(0.8)
        j = self.joints()
        got = (j.get("head_pitch_joint"), j.get("head_yaw_joint"))
        self._log(f"look_at -> pitch {got[0]*DEG:+.1f}deg yaw {got[1]*DEG:+.1f}deg "
                  f"(asked {p1*DEG:+.1f} / {y1*DEG:+.1f})")
        return got

    # -------------------------------------------------------------- grippers
    def grip(self, side, close01, timeout=20.0):
        """close01: 0.0 = fully OPEN, 1.0 = fully CLOSED.

        Polls the position readback instead of sleeping a fixed settle -- a
        fixed 1.5 s sleep was most of the measured 'motion time'.

        timeout is a bound on the READBACK settling, not on jaw travel. The jaws
        themselves are fast (2026-08-18: 14 -> 844 mm in 0.65 s), but the position
        topic can keep serving a stale value well past the move: the same day a
        close returned "827 -> 643 mm (3.04 s)" and the real resting value, polled
        later, was 14 mm. The old 3.0 s default made that stale number look like a
        settled measurement -- i.e. it silently reported the wrong jaw opening, which
        is the one number that decides "did we grasp anything". It breaks early once
        the readback reaches the target, so a healthy move still costs 0.65 s.
        """
        close01 = max(0.0, min(1.0, float(close01)))
        want_mm = (1.0 - close01) * GRIP_MAX_MM
        # 发之前先等 DDS 发现完成。命令话题是 volatile/best-effort，没有 latching，所以
        # 订阅者还没被发现时 publish() 就是**静默丢弃**。2026-08-14 实测：进程 attach 后
        # 3s 就 close，日志打 `before=None -> 827mm`（state 话题 20Hz 却一条没收到 ⇒ 发现
        # 没完成），夹爪一动不动；隔 70s 手动 publish 同一个值，8s 内 828->4mm。
        # `before is None` 就是这件事的可见迹象 —— 它不是"读不到"，是"整个话题还没通"。
        t_d = time.time()
        while time.time() - t_d < 2.0:
            if self._grip_pub[side].get_subscription_count() > 0 \
                    and self._grip_pos[side] is not None:
                break
            self.spin(0.05)
        before = self._grip_pos[side]
        if self._grip_pub[side].get_subscription_count() == 0:
            self._log(f"{side} gripper: 命令话题 0 个订阅者 —— g2_gripper_pc 没在跑，"
                      f"这一条 publish 会被丢掉")
        self._grip_pub[side].publish(self._F64(data=close01))
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(0.05)
            cur = self._grip_pos[side]
            if cur is not None and abs(cur - want_mm) <= GRIP_TOL_MM:
                break
        after = self._grip_pos[side]
        el = time.time() - t0
        if after is None:
            self._log(f"{side} gripper: NO readback -- g2_gripper_node is not "
                      f"running (ros2 launch g2_gripper_pc g2_gripper_pc.launch.py)")
        else:
            self._log(f"{side} gripper -> {'CLOSE' if close01 > 0.5 else 'OPEN'}"
                      f"  {before} -> {after} mm  ({el:.2f}s)")
        return before, after

    def grip_state(self, side):
        return self._grip_pos[side]

    # --------------------------------------------------------------- cameras
    def snap_cam(self, which, out, timeout=6.0):
        """Grab one frame from a wrist/chest camera, whichever transport it uses.

        Callers should not have to know that the right wrist is a ROS-topic device
        and the others are /dev/video. Returns the path, or None.
        """
        if which in CAM_TOPICS:
            return self.snap_ros_image(CAM_TOPICS[which], out, timeout)
        return self.snap_uvc(which, out)

    def snap_ros_image(self, topic, out, timeout=6.0):
        """Grab one sensor_msgs/Image off a topic and write it as JPEG.

        Used for the right-wrist D455. Not CompressedImage: realsense2_camera
        publishes raw rgb8 unless you run a republisher.
        """
        import cv2
        import numpy as np
        from sensor_msgs.msg import Image
        got = {}
        sub = self.node.create_subscription(
            Image, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.05)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            self._log(f"no frame on {topic} in {timeout:.0f}s")
            return None
        m = got["m"]
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            a = a[:, :, ::-1]                 # color_stats 等下游按 BGR 读
        cv2.imwrite(out, a)
        return out

    @staticmethod
    def snap_uvc(which, out):
        """Capture one UVC camera. MUST be MJPG or the capture hangs."""
        import cv2
        dev = CAMERAS[which]
        cap = cv2.VideoCapture(os.path.realpath(dev), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        frame = None
        for _ in range(8):                       # first frames are often black
            ok, f = cap.read()
            if ok:
                frame = f
        cap.release()
        if frame is None:
            return None
        cv2.imwrite(out, frame)
        return out

    def snap_zed(self, out, timeout=8.0):
        """Grab a ZED frame from the ROS graph -- the CORRECT way.

        Astrabot_ZED.service runs zed_wrapper, which holds the camera
        EXCLUSIVELY. Any direct pyzed open therefore fails with "CAMERA STREAM
        FAILED TO START". But the wrapper already publishes rectified RGB,
        registered depth, point clouds, IMU and full TF -- all reachable from the
        plain ROS interpreter with no venv. Use that.
        """
        import cv2
        import numpy as np
        from sensor_msgs.msg import CompressedImage
        got = {}
        sub = self.node.create_subscription(
            CompressedImage, ZED_RGB_COMPRESSED,
            lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.05)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            self._log("ZED: no frame on ROS topic; is Astrabot_ZED.service up?")
            return None
        img = cv2.imdecode(np.frombuffer(got["m"].data, np.uint8),
                           cv2.IMREAD_COLOR)
        cv2.imwrite(out, img)
        self._log(f"ZED frame {img.shape[1]}x{img.shape[0]} -> {out}")
        return out

    @staticmethod
    def snap_zed_pyzed(out):
        """Direct pyzed capture. ONLY works if Astrabot_ZED.service is STOPPED
        (the wrapper holds the camera exclusively). Prefer snap_zed()."""
        code = (
            "import sys,pyzed.sl as sl\n"
            "z=sl.Camera(); p=sl.InitParameters()\n"
            "p.camera_resolution=sl.RESOLUTION.HD720; p.camera_fps=30\n"
            "if z.open(p)!=sl.ERROR_CODE.SUCCESS: sys.exit('zed open failed')\n"
            "m=sl.Mat(); rt=sl.RuntimeParameters()\n"
            "for _ in range(6):\n"
            "    if z.grab(rt)==sl.ERROR_CODE.SUCCESS: z.retrieve_image(m, sl.VIEW.LEFT)\n"
            f"m.write({out!r}); z.close(); print('ok')\n")
        p = subprocess.run([DEPLOY_PY, "-c", code], capture_output=True,
                           text=True, timeout=90)
        return out if "ok" in (p.stdout or "") else None


# --------------------------------------------------------------- recording ----
class _NoStep:
    """--record off: same API, does nothing. Keeps the command bodies identical
    whether or not there is a camera, so the recorded path is the SAME code path
    that runs unrecorded -- no separate branch to rot."""

    seq = 0

    def __enter__(self):
        return self

    def __exit__(self, *e):
        return False

    def mark(self, label):
        pass

    def note(self, text):
        pass

    def fail(self, why):
        pass


def _step(a, action, params=None, robot=None):
    """Wrap one action in an experiment Step when --record was given.

    Deliberately imported lazily: xr1_experiment reaches out over SSH to the Mac,
    and `xr1.py pose` must not pay for that.
    """
    if not getattr(a, "record", False):
        return _NoStep()
    sys.path.insert(0, SCRIPTS)
    import xr1_experiment
    return xr1_experiment.Step(action, params, robot=robot,
                               label=getattr(a, "label", "") or "")


def _add_record_flags(p):
    p.add_argument("--record", action="store_true",
                   help="film this action with the Mac's external camera and "
                        "write an experiment record (starts recording first)")
    p.add_argument("--label", default="",
                   help="label for the run this action belongs to")


# ------------------------------------------------------------------- CLI ----
def cmd_status(a):
    os.execvp(sys.executable, [sys.executable,
                               os.path.join(SCRIPTS, "xr1_verify.py")]
              + (["--quick"] if a.quick else []))


def cmd_bringup(a):
    """Start whatever is missing. The G2 gripper driver is NOT a systemd
    service, so it is gone after every reboot -- and the RTC is dead, so a
    reboot is invisible unless you check /proc/uptime."""
    up = float(open("/proc/uptime").read().split()[0])
    print(f"  uptime {up/60:.1f} min | load {open('/proc/loadavg').read().split()[0]}")
    if up < 180:
        print("  NOTE: booted <3 min ago -- rates and DDS discovery are still "
              "settling; a low Hz reading now is not a fault.")

    have = subprocess.run("pgrep -f 'g2_gripper[_]node'", shell=True,
                          capture_output=True, text=True).returncode == 0
    if have:
        print("  g2_gripper_node already running")
    else:
        print("  g2_gripper_node MISSING -> launching")
        subprocess.Popen(
            "source /opt/ros/jazzy/setup.bash && "
            "source /opt/ros/astrabot/setup.bash && "
            "source /home/astrabot/gripper_ws/install/setup.bash && "
            "exec ros2 launch g2_gripper_pc g2_gripper_pc.launch.py",
            shell=True, executable="/bin/bash",
            stdout=open("/tmp/g2_gripper.log", "w"),
            stderr=subprocess.STDOUT, start_new_session=True)
        for _ in range(30):
            time.sleep(1)
            if subprocess.run("pgrep -f 'g2_gripper[_]node'", shell=True,
                              capture_output=True).returncode == 0:
                break
        print("  launched; log /tmp/g2_gripper.log")

    with XR1(verbose=False) as r:
        r.spin(2.0)
        for s in ("left", "right"):
            v = r.grip_state(s)
            print(f"  {s} gripper readback: "
                  f"{'%s mm  OK' % v if v is not None else 'STILL SILENT'}")
    print("  now run:  python3 xr1.py status")


def cmd_pose(a):
    with XR1(verbose=False) as r:
        p = r.pose()
        for side in ("left", "right"):
            print(f"  {side:5s} " + " ".join(
                "  nan " if v is None else f"{v:+.3f}" for v in p[side]))
        print(f"  neck  " + " ".join(
            "  nan " if v is None else f"{v:+.3f}" for v in p["neck"])
            + "   (pitch, yaw in rad)")
        for s in ("left", "right"):
            print(f"  {s} gripper  {r.grip_state(s)} mm  "
                  f"(840=open, ~5=closed)")
        for s in ("left", "right"):
            print(f"  {s} tcp z    {r.tcp_z(s)}")


def cmd_look(a):
    with XR1() as r:
        with _step(a, "look", {"pitch": a.pitch, "yaw": a.yaw}, robot=r):
            r.look_at(pitch_deg=a.pitch, yaw_deg=a.yaw)


def cmd_grip(a):
    val = {"open": 0.0, "close": 1.0}.get(a.value, None)
    if val is None:
        val = float(a.value)
    with XR1() as r:
        with _step(a, "grip", {"side": a.side, "value": val}, robot=r):
            r.grip(a.side, val)


def cmd_wave(a):
    with XR1() as r:
        with _step(a, "wave", {"side": a.side, "rounds": a.rounds}, robot=r) as st:
            q0 = r.raise_arm(a.side)
            st.mark("raised")
            r.wave(a.side, n=a.rounds)
            st.mark("waved")
            r.grip(a.side, 1.0)
            r.grip(a.side, 0.0)
            st.mark("gripped")
            r.lower_arm(a.side, q0)
            st.mark("lowered")


def cmd_home(a):
    with XR1() as r:
        with _step(a, "home", {"which": a.which}, robot=r):
            r.home(a.which)


def cmd_demo(a):
    t0 = time.time()
    marks = []
    with XR1() as r:
        setup = time.time() - t0
        for rnd in range(a.rounds):
            for side in ("right", "left"):
                # One Step per arm-turn, not one for the whole demo: each turn is
                # the unit a human wants to re-watch and judge separately.
                with _step(a, "demo_turn", {"round": rnd, "side": side},
                           robot=r) as st:
                    q0 = r.raise_arm(side)
                    marks.append((f"r{rnd} {side} raise", time.time() - t0))
                    st.mark("raise")
                    r.wave(side, n=2)
                    marks.append((f"r{rnd} {side} wave", time.time() - t0))
                    st.mark("wave")
                    r.grip(side, 1.0)
                    r.grip(side, 0.0)
                    marks.append((f"r{rnd} {side} grip", time.time() - t0))
                    st.mark("grip")
                    r.lower_arm(side, q0)
                    marks.append((f"r{rnd} {side} lower", time.time() - t0))
                    st.mark("lower")
    total = time.time() - t0
    print("\n=== TIMING ===")
    prev = setup
    for label, t in marks:
        print(f"  {t:7.3f}s (+{t-prev:5.2f})  {label}")
        prev = t
    print(f"\n  one-time setup {setup:.2f}s | motion {total-setup:.2f}s | "
          f"total {total:.2f}s | {len(marks)} phases")
    print(f"  as separate shell calls: +{setup:.1f}s x {len(marks)} = "
          f"{setup*len(marks):.0f}s of pure setup, plus one LLM round-trip each.")


def cmd_rec(a):
    """Front door for the recording pipeline: camera plumbing lives in
    xr1_cam.py, experiment records and movie assembly in xr1_experiment.py.
    Dispatched as a subprocess so a broken SSH/ffmpeg path can never take down
    the control layer."""
    cam = {"setup": "install", "doctor": "doctor", "status": "status",
           "test": "selftest", "quit": "quit"}
    if a.recmd in cam:
        script, argv = "xr1_cam.py", [cam[a.recmd]]
    else:
        script, argv = "xr1_experiment.py", [a.recmd]
    return subprocess.call([sys.executable, os.path.join(SCRIPTS, script)]
                           + argv + list(a.rest))


def cmd_snap(a):
    out = a.out
    os.makedirs(out, exist_ok=True)
    want = a.which or (["zed", "chest", "lwrist", "rwrist"] if a.all else ["zed"])
    # rwrist(D455) 也走 ROS，所以它和 zed 一样需要节点；纯 UVC 的那几个不需要
    # （XR1() 的构造依赖 /joint_states，不必要时别碰它）
    need_node = "zed" in want or any(w in CAM_TOPICS for w in want)
    r = XR1(verbose=True) if need_node else None
    try:
        for w in want:
            p = os.path.join(out, f"{w}.jpg")
            got = (r.snap_zed(p) if w == "zed" else
                   r.snap_cam(w, p) if r else XR1.snap_uvc(w, p))
            print(f"  {w:7s} -> {got or 'FAILED'}")
    finally:
        if r:
            r.close()


def cmd_blocks(a):
    j = "/tmp/blocks.json"
    print("--- ZED perception (deploy venv, py3.10) ---")
    subprocess.run([DEPLOY_PY, os.path.join(SCRIPTS, "zed_perception.py"),
                    "--json", j] + (["--save-viz"] if a.viz else []))
    print("--- to base_link (ROS py3.12) ---")
    subprocess.run([sys.executable, os.path.join(SCRIPTS, "blocks_to_base.py"),
                    "--json", j])


def main():
    ap = argparse.ArgumentParser(
        description="XR1 unified control. Needs ROS_DOMAIN_ID=12.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("status", help="full state probe")
    p.add_argument("--quick", action="store_true")
    p.set_defaults(fn=cmd_status)

    sub.add_parser("bringup", help="start anything missing (esp. the gripper "
                                   "driver, which dies on every reboot)"
                   ).set_defaults(fn=cmd_bringup)

    sub.add_parser("pose", help="current joints / grippers / tcp").set_defaults(
        fn=cmd_pose)

    p = sub.add_parser("look", help="head pitch/yaw in DEGREES (limit +-40)")
    p.add_argument("pitch", type=float)
    p.add_argument("yaw", type=float, nargs="?", default=0.0)
    _add_record_flags(p)
    p.set_defaults(fn=cmd_look)

    p = sub.add_parser("grip", help="open | close | 0.0-1.0")
    p.add_argument("side", choices=["left", "right"])
    p.add_argument("value")
    _add_record_flags(p)
    p.set_defaults(fn=cmd_grip)

    p = sub.add_parser("wave", help="raise + wave + gripper + lower, one arm")
    p.add_argument("side", choices=["left", "right"])
    p.add_argument("--rounds", type=int, default=2)
    _add_record_flags(p)
    p.set_defaults(fn=cmd_wave)

    p = sub.add_parser("home", help="arm joints to 0")
    p.add_argument("which", nargs="?", default="both",
                   choices=["left", "right", "both"])
    _add_record_flags(p)
    p.set_defaults(fn=cmd_home)

    p = sub.add_parser("demo", help="both arms alternating, timed")
    p.add_argument("--rounds", type=int, default=2)
    _add_record_flags(p)
    p.set_defaults(fn=cmd_demo)

    p = sub.add_parser("snap", help="capture cameras")
    p.add_argument("--which", nargs="*",
                   choices=["zed", "chest", "lwrist", "rwrist"])
    p.add_argument("--all", action="store_true")
    p.add_argument("--out", default="/tmp/xr1_snaps")
    p.set_defaults(fn=cmd_snap)

    p = sub.add_parser("blocks", help="ZED -> table plane -> blocks in base_link")
    p.add_argument("--viz", action="store_true")
    p.set_defaults(fn=cmd_blocks)

    p = sub.add_parser(
        "rec", help="experiment recording: Mac camera + per-action records + movie")
    p.add_argument("recmd", choices=["setup", "doctor", "status", "test", "quit",
                                     "new", "ls", "show", "movie", "report",
                                     "end", "demo-step"])
    p.add_argument("rest", nargs=argparse.REMAINDER,
                   help="passed through, e.g. --label 抓积木")
    p.set_defaults(fn=cmd_rec)

    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main() or 0)
