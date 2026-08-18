#!/usr/bin/env python3
"""Measure the FK-vs-physical gripper-pad offset against the camera.

Exp 17 hit its commanded Cartesian target to 0.8 mm and still closed on air, so
the error has to live in the tool-frame model (where FK thinks the pads are)
rather than in IK, the gates, or the actuators. Nothing else in this workspace
observes that error, because every gate compares FK against FK.

This compares FK against the IMAGE: it projects the FK pad points into the frame
and independently finds the physical pads by their orange colour.

    pad_offset_measure.py FRAME_DIR FK_JSON      one pose, image plane only
    pad_offset_measure.py solve FRAME_DIR...     many poses, full tool-frame vector
    pad_offset_measure.py --check                offline self-check

`FK_JSON` is the stdout of `xr1-vision fk Q1..Q7` for the pose in that frame;
`solve` runs `xr1-vision fk` itself for every frame it is given.

One pose cannot separate a constant tool-frame offset from a rotation error in
the model, which is why `solve` exists: a constant offset gives the SAME vector
in the tool frame at every wrist orientation, a rotation error does not.
"""
import json
import subprocess
import sys

import numpy as np
from PIL import Image

FK_BIN = "/home/astrabot/workspace/target/release/xr1-vision"
JOINTS = ["right_arm_%d_joint" % i for i in range(1, 8)]
# Deliberately far larger than any plausible model error: we are measuring a
# residual of tens of pixels, so a 250 px candidate is a different object.
SEARCH_PX = 150.0


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


class Frame:
    """One observation: intrinsics, image-time TF, RGB and (if present) depth."""

    def __init__(self, frame_dir):
        self.dir = frame_dir
        self.state = json.load(open(f"{frame_dir}/state.json"))
        info = json.load(open(f"{frame_dir}/camera_info.json"))
        k = info["k"]
        self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]
        # tf is camera -> base (target base_link, source the optical frame).
        tf = self.state["tf"]
        self.r_bc = quat_to_matrix(*tf["rotation_xyzw"])
        self.t_bc = np.array(tf["translation_m"])
        self.rgb = np.asarray(Image.open(f"{frame_dir}/rgb.png").convert("RGB"))

    def joints(self):
        p = self.state["joint_state"]["positions_rad"]
        return [p[n] for n in JOINTS]

    def project(self, p_base):
        p = self.r_bc.T @ (np.asarray(p_base) - self.t_bc)
        if p[2] <= 0:
            raise SystemExit(f"point {p_base} is behind the camera")
        return self.fx * p[0] / p[2] + self.cx, self.fy * p[1] / p[2] + self.cy, p[2]

    def unproject(self, u, v, depth):
        p_c = np.array([(u - self.cx) / self.fx * depth,
                        (v - self.cy) / self.fy * depth, depth])
        return self.r_bc @ p_c + self.t_bc

    def depth_at(self, u, v, half=4):
        """Median finite depth in a small window. None if nothing is valid there."""
        d = np.load(f"{self.dir}/depth.npy")
        y, x = int(round(v)), int(round(u))
        win = d[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
        good = win[np.isfinite(win) & (win > 0.05)]
        return float(np.median(good)) if good.size else None

    def pads_near(self, pu, pv):
        """The pad blobs close to a prediction, plus what was rejected.

        Orange is not unique in this scene: a bowl of fake fruit on the left has a
        plastic orange BIGGER than either pad (2701 px vs 810/337 on frame
        20260818-120701), so "the two largest orange blobs" picks the fruit and
        reports a 216 mm offset.
        """
        blobs = orange_pad_centroids(self.rgb)
        near = [b for b in blobs
                if ((b[1] - pu) ** 2 + (b[2] - pv) ** 2) ** 0.5 <= SEARCH_PX]
        return near, len(blobs) - len(near)


def fk_of(frame):
    out = subprocess.run([FK_BIN, "fk"] + ["%.9f" % v for v in frame.joints()],
                         capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def measure(frame, fk):
    """Image-plane offset between the FK pad midpoint and the physical pads.

    The two pads bracket the grasp centre, so their midpoint IS the physical
    grasp point, and it is far less sensitive to how much of each pad the camera
    sees than either centroid alone. It is still biased: at a typical viewing
    angle the near pad shows 810 px and the far one 337, so the midpoint leans
    toward the near jaw. Treat the magnitude as approximate and the SIGN as solid.
    """
    mid = fk["pad_midpoint_base_m"]
    pu, pv, depth = frame.project(mid)
    near, rejected = frame.pads_near(pu, pv)
    if len(near) < 2:
        return {"frame": frame.state["frame_id"], "ok": False,
                "reason": f"need both pads near the prediction, found {len(near)}",
                "blobs_rejected_as_other_objects": rejected}
    (_, u1, v1), (_, u2, v2) = near[0], near[1]
    mu, mv = (u1 + u2) / 2, (v1 + v2) / 2
    du, dv = mu - pu, mv - pv
    # mm per pixel at the pad's own depth, from the intrinsics -- not the
    # 1.84 px/mm constant, which was measured at the table plane.
    mm_u, mm_v = 1000 * depth / frame.fx, 1000 * depth / frame.fy
    return {"frame": frame.state["frame_id"], "ok": True,
            "fk_pad_midpoint_base_m": [round(v, 5) for v in mid],
            "fk_projection_uv": [round(pu, 1), round(pv, 1)],
            "fk_pad_depth_m": round(depth, 4),
            "physical_pad_midpoint_uv": [round(mu, 1), round(mv, 1)],
            "offset_px": [round(du, 1), round(dv, 1)],
            "offset_mm_image_plane": [round(du * mm_u, 1), round(dv * mm_v, 1)],
            "offset_magnitude_mm": round(((du * mm_u) ** 2 + (dv * mm_v) ** 2) ** 0.5, 1),
            "blobs_rejected_as_other_objects": rejected,
            "note": "image-plane only; the component along the viewing ray is invisible here",
            "_uv": (mu, mv)}


def solve_tool_offset(frame_dirs):
    """Multi-pose solve: the same offset expressed in the TOOL frame each time.

    Needs depth, so it only works on frames that saved `depth.npy`. Per pose the
    physical pad midpoint is unprojected at its own measured depth, so unlike
    `measure` this includes the component along the viewing ray.
    """
    rows = []
    for d in frame_dirs:
        frame = Frame(d)
        fk = fk_of(frame)
        m = measure(frame, fk)
        if not m["ok"]:
            rows.append({"frame": m["frame"], "ok": False, "reason": m["reason"]})
            continue
        mu, mv = m["_uv"]
        depth = frame.depth_at(mu, mv)
        if depth is None:
            rows.append({"frame": m["frame"], "ok": False,
                         "reason": "no valid depth on the pads"})
            continue
        phys = frame.unproject(mu, mv, depth)
        delta_base = phys - np.array(fk["pad_midpoint_base_m"])
        r_tool = np.array(fk["tool_rotation_base_rowmajor"])
        delta_tool = r_tool.T @ delta_base
        rows.append({"frame": m["frame"], "ok": True,
                     "pad_depth_m": round(depth, 4),
                     "offset_mm_image_plane": m["offset_mm_image_plane"],
                     "delta_tool_mm": [round(1000 * v, 1) for v in delta_tool]})
    good = np.array([r["delta_tool_mm"] for r in rows if r["ok"]])
    out = {"poses": rows, "n_solved": len(good)}
    if len(good) >= 2:
        out["delta_tool_mm_mean"] = [round(v, 1) for v in good.mean(axis=0)]
        out["delta_tool_mm_std"] = [round(v, 1) for v in good.std(axis=0)]
        out["reading"] = ("std small relative to mean on an axis => a constant "
                          "tool-frame offset on that axis; std comparable to mean "
                          "=> a rotation error, and adding a constant will not fix it")
    return out


def demo():
    """Offline self-check: projection round-trip, and the colour discrimination."""
    class FakeFrame(Frame):
        def __init__(self):
            self.fx = self.fy = 700.0
            self.cx, self.cy = 640.0, 360.0
            # A deliberately non-identity rotation: with identity, project() and
            # unproject() would agree even if one of them transposed the matrix.
            self.r_bc = quat_to_matrix(-0.7071067811865476, 0.0, 0.0, 0.7071067811865476)
            self.t_bc = np.array([0.05, 0.0, 1.34])

    f = FakeFrame()
    p = f.t_bc + f.r_bc @ np.array([0.10, -0.05, 1.0])   # 1 m in front of the lens
    u, v, d = f.project(p)
    assert np.allclose(f.unproject(u, v, d), p, atol=1e-9), (f.unproject(u, v, d), p)

    def patch(rgb_value):
        img = np.zeros((40, 40, 3), dtype=np.uint8)
        img[:, :] = rgb_value
        return len(orange_pad_centroids(img))

    assert patch([231, 124, 31]) == 1, "orange pad must be found"
    assert patch([179, 184, 81]) == 0, "yellow block must not read as a pad"
    print("self-check OK")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args or "--check" in args:
        demo() if "--check" in args else print(__doc__)
    elif args[0] == "solve":
        print(json.dumps(solve_tool_offset(args[1:]), indent=1))
    else:
        frame = Frame(args[0])
        result = measure(frame, json.load(open(args[1])))
        result.pop("_uv", None)
        print(json.dumps(result, indent=1))
