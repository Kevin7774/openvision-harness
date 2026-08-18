"""Log D455 frame-to-frame change with timestamps, so an arm jog can be correlated.

Read-only. Deadline inside the loop (PITFALLS 35: `timeout N python3` does not bound rclpy).
Writes one JSON line per frame: {t, mad} where mad = mean |Delta| over the grey frame.
Structured light / rolling noise gives a nonzero floor, so the answer is the RATIO
between the moving window and the still baseline, never the absolute value.
"""
import json, os, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = "/home/astrabot/workspace/zed_ws/experiments/d455_which_arm/watch.jsonl"
DEADLINE_S = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


class W(Node):
    def __init__(s):
        super().__init__("d455_watch")
        s.f = None; s.n = 0
        s.create_subscription(Image, "/right_wrist/d455/color/image_raw", s.cb, 5)

    def cb(s, m):
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
        s.f = a[::4, ::4].mean(axis=2).astype(np.float32)   # 1/4 downsample, grey
        s.n += 1


rclpy.init(); w = W(); t0 = time.time()
log = open(OUT, "w", buffering=1)
prev, seen = None, 0
while time.time() - t0 < DEADLINE_S:
    rclpy.spin_once(w, timeout_sec=0.2)
    if w.n == seen:
        continue
    seen = w.n
    if prev is not None:
        log.write(json.dumps({"t": round(time.time(), 3),
                              "mad": round(float(np.abs(w.f - prev).mean()), 4)}) + "\n")
    prev = w.f
print(f"{seen} 帧写入 {OUT}"); sys.stdout.flush(); os._exit(0)
