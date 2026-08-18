#!/usr/bin/env python3
"""Measure the FK-vs-physical gripper-pad offset from one observation frame.

Exp 17 hit its commanded Cartesian target to 0.8 mm and still closed on air, so
the error has to live in the tool-frame model (where FK thinks the pads are)
rather than in IK, the gates, or the actuators. Nothing in the harness observes
that error, because every gate compares FK against FK.

This compares FK against the IMAGE: it projects the FK pad points into the frame
and independently finds the physical pads by their orange colour. The gap between
the two projections is the model error, in pixels and (at the pad's own depth) in
millimetres. Run it on two different poses -- a constant tool-frame offset gives
the same tool-frame vector both times, a rotation error does not.

usage: pad_offset_measure.py FRAME_DIR FK_JSON
  FK_JSON is the stdout of `xr1-vision fk Q1..Q7` for the pose in that frame.
"""
import json
import sys

import numpy as np
from PIL import Image


def quat_to_matrix(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (x * w + y * z), 1 - 2 * (x * x + y * y)],
    ])


def orange_pad_centroids(rgb):
    """The two gripper pads, found by colour. Returns centroids sorted by area."""
    r, g, b = (rgb[:, :, i].astype(np.int16) for i in range(3))
    # Measured on 20260818-120701: pads sit near (231,124,31). Orange is the only
    # thing in the workspace with R clearly above G and G clearly above B; the
    # yellow block has R ~= G, so the r>g margin is what separates them.
    mask = (r > g + 55) & (g > b + 30) & (r > 120)
    labels = np.zeros(mask.shape, dtype=np.int32)
    current, blobs = 0, []
    for seed in zip(*np.nonzero(mask)):
        if labels[seed]:
            continue
        current += 1
        stack, pixels = [seed], []
        labels[seed] = current
        while stack:
            y0, x0 = stack.pop()
            pixels.append((y0, x0))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    y1, x1 = y0 + dy, x0 + dx
                    if 0 <= y1 < mask.shape[0] and 0 <= x1 < mask.shape[1] \
                            and mask[y1, x1] and not labels[y1, x1]:
                        labels[y1, x1] = current
                        stack.append((y1, x1))
        if len(pixels) >= 150:
            ys, xs = zip(*pixels)
            blobs.append((len(pixels), float(np.mean(xs)), float(np.mean(ys))))
    return sorted(blobs, reverse=True)


frame_dir, fk_json = sys.argv[1], sys.argv[2]
state = json.load(open(f"{frame_dir}/state.json"))
info = json.load(open(f"{frame_dir}/camera_info.json"))
fk = json.load(open(fk_json))

k = info["k"]
fx, fy, cx, cy = k[0], k[4], k[2], k[5]
tf = state["tf"]
# tf is camera -> base (target base_link, source the optical frame), so invert it.
r_bc = quat_to_matrix(*tf["rotation_xyzw"])
t_bc = np.array(tf["translation_m"])
r_cb, t_cb = r_bc.T, -r_bc.T @ t_bc


def project(p_base):
    p = r_cb @ np.asarray(p_base) + t_cb
    if p[2] <= 0:
        raise SystemExit(f"point {p_base} is behind the camera")
    return fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy, p[2]


fixed = fk["fixed_pad_inner_base_m"]
moving = fk["moving_pad_inner_base_m"]
mid = fk["pad_midpoint_base_m"]
pu, pv, depth = project(mid)

rgb = np.asarray(Image.open(f"{frame_dir}/rgb.png").convert("RGB"))
blobs = orange_pad_centroids(rgb)
print(json.dumps({
    "frame": state["frame_id"],
    "fk_pad_midpoint_base_m": [round(v, 5) for v in mid],
    "fk_projection_uv": [round(pu, 1), round(pv, 1)],
    "fk_pad_depth_m": round(depth, 4),
    "fk_fixed_uv": [round(v, 1) for v in project(fixed)[:2]],
    "fk_moving_uv": [round(v, 1) for v in project(moving)[:2]],
    "orange_blobs": [{"area": a, "uv": [round(u, 1), round(v, 1)]} for a, u, v in blobs],
}, indent=1))

# Orange is not unique in this scene: there is a bowl of fake fruit on the left
# whose plastic orange is a BIGGER blob than either pad (area 2701 vs 810/337 on
# frame 20260818-120701), so "the two largest orange blobs" picks the fruit and
# reports a 216 mm offset. Restrict to the neighbourhood of the prediction -- we
# are measuring a residual of tens of pixels, so a 250 px candidate is a
# different object, not a model error. SEARCH_PX is deliberately far larger than
# any plausible offset, and everything rejected is printed above.
SEARCH_PX = 150.0
near = [b for b in blobs if ((b[1] - pu) ** 2 + (b[2] - pv) ** 2) ** 0.5 <= SEARCH_PX]
print(json.dumps({"blobs_within_%dpx" % SEARCH_PX: len(near),
                  "blobs_rejected_as_other_objects": len(blobs) - len(near)}))
if len(near) < 2:
    raise SystemExit(f"need both pads visible near the prediction, found {len(near)}")

# The two pads bracket the grasp centre, so their midpoint IS the physical grasp
# point -- and it is far less sensitive to how much of each pad the camera sees
# than either pad centroid on its own. It is still biased: at this viewing angle
# the near pad shows 810 px and the far one 337, so the midpoint leans toward the
# near jaw. Treat the magnitude as approximate and the SIGN as solid.
(_, u1, v1), (_, u2, v2) = near[0], near[1]
mu, mv = (u1 + u2) / 2, (v1 + v2) / 2
du, dv = mu - pu, mv - pv
# mm per pixel at the pad's own depth, from the intrinsics -- not the 1.84 px/mm
# constant, which was measured at the table plane.
mm_u, mm_v = 1000 * depth / fx, 1000 * depth / fy
print(json.dumps({
    "physical_pad_midpoint_uv": [round(mu, 1), round(mv, 1)],
    "offset_px": [round(du, 1), round(dv, 1)],
    "offset_mm_image_plane": [round(du * mm_u, 1), round(dv * mm_v, 1)],
    "offset_magnitude_mm": round(((du * mm_u) ** 2 + (dv * mm_v) ** 2) ** 0.5, 1),
    "note": "image-plane only; the component along the viewing ray is invisible here",
}, indent=1))


def demo():
    """Self-check: the projection must round-trip a point at a known depth."""
    p = np.array([0.43, -0.14, 0.89])
    u, v, d = project(p)
    ray = np.linalg.inv(r_cb) @ (np.array([(u - cx) / fx * d, (v - cy) / fy * d, d]) - t_cb)
    assert np.allclose(ray, p, atol=1e-9), (ray, p)
    # A pure orange patch must be found; a pure yellow one must not.
    def patch(rgb_value):
        img = np.zeros((40, 40, 3), dtype=np.uint8)
        img[:, :] = rgb_value
        return len(orange_pad_centroids(img))
    assert patch([231, 124, 31]) == 1, "orange pad must be found"
    assert patch([179, 184, 81]) == 0, "yellow block must not read as a pad"
    print("self-check OK")


if __name__ == "__main__" and "--check" in sys.argv:
    demo()
