"""Grab one frame from the surviving mono wrist camera and characterise it.

The point is the orange gripper pads: a camera really bolted to a wrist sees that
gripper's two pads. Nothing else on this machine can tell the two DECXIN cameras
apart -- same VID:PID, same hardcoded ID_SERIAL -- so a frame is the only evidence.
Must open MJPG or capture hangs on this box. Deadline inside the loop (PITFALLS 35).
"""
import os, sys, time
import cv2, numpy as np

DEV = "/dev/l_arm_cam"
cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print(f"cannot open {DEV}"); sys.stdout.flush(); os._exit(1)

frame, t0, n = None, time.time(), 0
while time.time() - t0 < 12.0:
    ok, f = cap.read()
    n += 1
    if ok and f is not None and f.size:
        frame = f
        if n > 8:          # let auto-exposure settle
            break
cap.release()
if frame is None:
    print(f"opened but no frame in 12 s ({n} reads)"); sys.stdout.flush(); os._exit(2)

h, w = frame.shape[:2]
print(f"got {w}x{h} after {n} reads")
cv2.imwrite("wrist.png", frame)

# orange pad detector: HSV hue ~5-25, high saturation
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
orange = cv2.inRange(hsv, (5, 120, 80), (25, 255, 255))
orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
frac = orange.mean() / 255
print(f"orange pixels: {100*frac:.2f}%")
nlab, lab, stats, cent = cv2.connectedComponentsWithStats(orange, 8)
blobs = sorted([(stats[i, cv2.CC_STAT_AREA], cent[i]) for i in range(1, nlab)],
               reverse=True, key=lambda b: b[0])[:5]
for a, c in blobs:
    if a > 200:
        print(f"  blob area {a:6d} px  centre ({c[0]:6.1f},{c[1]:6.1f})  "
              f"= ({100*c[0]/w:4.1f}%, {100*c[1]/h:4.1f}% of frame)")
print(f"mean brightness {frame.mean():.1f}  (near 0 = lens blocked / dark)")
cv2.imwrite("wrist_orange.png", cv2.bitwise_and(frame, frame, mask=orange))
print("wrote wrist.png + wrist_orange.png")
sys.stdout.flush(); os._exit(0)
