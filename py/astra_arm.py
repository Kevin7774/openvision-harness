#!/usr/bin/env python3
"""Rate-limited joint control for the AstraBot XR1 arms, head, grippers and lift.

This is the layer an agent writes control programs against. It exists because the
only arm command channel on this robot is a bare
`/astrabot_arm_forward_position_controller/commands`
(std_msgs/Float64MultiArray, 14 positions in radians), driven by a
`forward_command_controller/ForwardCommandController`. That controller does *no*
interpolation: whatever you publish becomes the setpoint on the very next 200 Hz
cycle, so publishing a target that is 1 rad away from the current pose is a
command to slam there as fast as the drives allow. Everything below exists to
make that channel safe to use from a program that is being written by a model:

  * targets are always interpolated from the *measured* pose with a
    raised-cosine profile and a per-joint velocity cap (default 0.25 rad/s
    against a hardware max_vel of 6-9 rad/s),
  * targets are clamped to the URDF joint limits, read from the live
    /robot_state_publisher `robot_description` at start-up,
  * the joints you are not commanding are held at their measured position,
    because the controller takes all 14 values at once and omitting them would
    command zero,
  * nothing moves if /joint_states is stale, if the displacement exceeds a
    per-call ceiling, or if another node is already streaming commands,
  * dry_run prints the exact target vector instead of publishing it.

Why publish to the controller topic directly: the vendor's own
`astrabot_actuator_sdk` node does exactly the same thing. It drives the lift and
grippers over its own CAN link, which is why those go through its topics instead.

ARBITRATION -- read this before adding a third writer.

Three things can publish to ARM_CMD_TOPIC, and the controller is a plain
ForwardCommandController: it consumes whatever arrived last, at 200 Hz, with no
interpolation and no ownership check.

    astrabot_actuator_sdk   idle at 0 Hz unless a canned motion is playing
                            (hello/byebye/cheer/... from astrabot_motion_list.yaml)
    astrabot_mrt            the OCS2 MPC tracker, part of Astrabot_Mpc.service.
                            Also idle until something publishes a reference.
    this module             only while a move() is in flight

There *is* an arbiter -- `astrabot_arbitration`, launched with the MPC stack --
but it does not arbitrate this topic. It sits one level up: it takes
`/reference/cmd` (std_msgs/String, JSON) and `/reference/pose`, serves
`astrabot_controller_interface/srv/TrajectoryCommand`, and publishes the selected
mode on `/astrabot/ctrlmode`. So the vendor's high-level path is

    /reference/cmd -> astrabot_arbitration -> astrabot_mpc/astrabot_mrt -> ARM_CMD_TOPIC

and this module is a peer of that whole chain, not a client of it. Use the
vendor path when you want Cartesian/pose goals with collision awareness; use
this module when you want a bounded, inspectable joint move you can reason about.
Do not use both at once.

`_assert_channel_idle` checks for a competing stream before moving and names the
node it saw, but it cannot stop one that starts midway. Astrabot_Mpc.service is
enabled, so MRT is present after every boot -- present but silent is fine; the
check is against traffic, not against existence.

Usage as a library:

    from astra_arm import Robot

    with Robot() as bot:
        print(bot.joints())                        # measured pose, by name
        bot.move({'left_arm_2': 0.15}, speed=0.2)  # everything else held
        bot.home('left')
        bot.gripper('left', 0.5)
        bot.look_at(pitch=0.2, yaw=-0.3)           # needs the neck controller

Usage from the shell (see --help):

    astra_arm.py --state
    astra_arm.py --move left_arm_2=0.15 --dry-run
    astra_arm.py --home left --speed 0.15

Safety notes for whoever runs this:
  * `home` for this robot is all-zeros -- every canned motion in
    astrabot_motion_list.yaml starts and ends there.
  * the lift (`set_height`) and gripper scales are NOT verified. See the
    docstrings on those two methods before trusting them.
  * keep the physical e-stop in reach. Nothing here can stop a mechanism that
    is already moving under the vendor SDK's control.
"""

import argparse
import math
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

ARM_CMD_TOPIC = "/astrabot_arm_forward_position_controller/commands"
NECK_CMD_TOPIC = "/astrabot_neck_forward_controller/commands"
HEIGHT_TOPIC = "/astrabot/height"
GRIPPER_TOPIC = {
    "left": "/rm_left/rm_driver/teleop_gripper_float",
    "right": "/rm_right/rm_driver/teleop_gripper_float",
}

# Order is fixed by astrabot_xr1_controller.yaml and must match exactly: the
# controller maps array index -> joint, with no names in the message.
ARM_JOINTS = [f"left_arm_{i}_joint" for i in range(1, 8)] + \
             [f"right_arm_{i}_joint" for i in range(1, 8)]
NECK_JOINTS = ["head_pitch_joint", "head_yaw_joint"]


def _canonical_joints(targets):
    """Accept 'left_arm_4' as well as 'left_arm_4_joint'.

    The URDF names all carry the _joint suffix, but it is the kind of detail that
    is easy to drop when writing a move by hand, and dropping it used to fail with
    a bare "not arm joints" that did not say why.
    """
    out = {}
    for name, value in targets.items():
        if name not in ARM_JOINTS and f"{name}_joint" in ARM_JOINTS:
            name = f"{name}_joint"
        out[name] = value
    return out

# Fallback limits, read off the live robot_description on 2026-08-05. Used only
# if the URDF cannot be fetched; the live parse takes precedence.
FALLBACK_LIMITS = {
    "left_arm_1_joint": (-3.1, 3.1), "right_arm_1_joint": (-3.1, 3.1),
    "left_arm_2_joint": (-0.174, 3.05), "right_arm_2_joint": (-3.05, 0.1744),
    "left_arm_3_joint": (-3.1, 3.1), "right_arm_3_joint": (-3.1, 3.1),
    "left_arm_4_joint": (-0.139, 2.355), "right_arm_4_joint": (-2.3, 0.0),
    "left_arm_5_joint": (-3.1, 3.1), "right_arm_5_joint": (-3.1, 3.1),
    "left_arm_6_joint": (-1.57, 1.57), "right_arm_6_joint": (-1.5, 1.5),
    "left_arm_7_joint": (-3.1, 3.1), "right_arm_7_joint": (-3.1, 3.1),
    # Head: +-0.698132 rad (40 deg) from GEAR_ANGLE_MAX in the vendor
    # astrabot_fd_sdk/AstrabotFdSm45bl.hpp, and confirmed by eye on the hardware
    # 2026-08-06. NOT the +-3.1 the URDF shipped -- that was a placeholder, and
    # commanding it drives the neck servo into its mechanical stop.
    "head_pitch_joint": (-0.698132, 0.698132),
    "head_yaw_joint": (-0.698132, 0.698132),
}

CMD_RATE_HZ = 100.0        # command stream rate; controller runs at 200 Hz
DEFAULT_SPEED = 0.25       # rad/s per joint. Hardware max_vel is 6.0-9.0 rad/s.
MAX_SPEED = 1.0            # refuse anything faster than this
MAX_STEP_RAD = 1.8         # refuse a single call that moves a joint further
STATE_STALE_S = 0.3        # refuse to move on /joint_states older than this
DISCOVERY_WAIT_S = 8.0     # how long to wait for the FIRST /joint_states at startup
                           # (DDS discovery only -- see the note in __init__)
ARRIVAL_TOL_RAD = 0.05     # post-move verification tolerance


class MotionRefused(RuntimeError):
    """A safety precondition failed. Nothing was published."""


def _raised_cosine(n):
    """Time scaling from 0 to 1 over n steps, with zero slope at both ends.

    A linear ramp would step-change velocity at start and stop, which on a
    position-forwarding controller shows up as a jolt. This keeps acceleration
    bounded without needing a full trajectory generator.
    """
    if n <= 1:
        return [1.0]
    return [0.5 * (1.0 - math.cos(math.pi * i / (n - 1))) for i in range(n)]


class Robot:
    """Handle on the robot's actuators. Use as a context manager."""

    def __init__(self, node_name="astra_arm", init_ros=True):
        self._owns_ros = init_ros and not rclpy.ok()
        if self._owns_ros:
            rclpy.init(args=None)
        self.node = Node(node_name)

        # joint_states is published at 200 Hz best-effort; we only ever want the
        # newest one, so depth 1.
        self._state = None
        self._state_t = 0.0
        self.node.create_subscription(
            JointState, "/joint_states", self._on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # RELIABLE is compatible with the controller's BEST_EFFORT subscription
        # (stronger offer than requested) and keeps the final setpoint of a ramp
        # from being silently dropped.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._arm_pub = self.node.create_publisher(Float64MultiArray, ARM_CMD_TOPIC, qos)
        self._neck_pub = self.node.create_publisher(Float64MultiArray, NECK_CMD_TOPIC, qos)
        self._height_pub = self.node.create_publisher(Float64, HEIGHT_TOPIC, qos)
        self._grip_pub = {s: self.node.create_publisher(Float64, t, qos)
                          for s, t in GRIPPER_TOPIC.items()}

        self.limits = self._load_limits()
        # This waits for DDS *discovery*, which is a different thing from the
        # per-motion staleness guard (STATE_STALE_S) and must not be confused with
        # it: stretching this timeout weakens nothing, because every move still
        # re-checks that the data it is about to act on is < 0.3 s old.
        # 1.5 s was too tight. On this box under sustained load (load average ~45
        # is its normal steady state -- zed_node, nav2, mpc and mrt alone are ~4
        # cores) the first message measured 2.56 s. The old timeout turned that
        # into "is Astrabot_Controller.service up?", which points the reader at a
        # dead controller when the controller is in fact publishing at 137-169 Hz.
        waited = 0.0
        while self._state is None and waited < DISCOVERY_WAIT_S:
            self._spin(0.5)
            waited += 0.5
        if self._state is None:
            raise MotionRefused(
                f"no /joint_states within {DISCOVERY_WAIT_S:.1f} s -- check "
                f"ROS_DOMAIN_ID=12 first, then whether the topic has traffic "
                f"(`ros2 topic hz /joint_states`); a live-but-slow graph looks "
                f"identical to a dead one from in here")

    # ---------------------------------------------------------------- plumbing

    def _on_state(self, msg):
        self._state = dict(zip(msg.name, msg.position))
        self._state_t = time.time()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _load_limits(self):
        """Read joint limits from the live robot_description.

        Falls back to the table baked in above rather than to "no limits": an
        unclamped target on this controller is a collision.
        """
        try:
            from rcl_interfaces.srv import GetParameters
            cli = self.node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
            if not cli.wait_for_service(timeout_sec=3.0):
                raise TimeoutError("robot_state_publisher not answering")
            fut = cli.call_async(GetParameters.Request(names=["robot_description"]))
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
            urdf = fut.result().values[0].string_value
            # Strip XML comments before scanning. The head joints carry a long
            # comment documenting the +-40 deg edit, which pushed their <limit>
            # tag past the window below -- so the head silently looked
            # unmodelled and look_at() refused with a misleading "arm-only
            # bring-up" message while the limits were in fact present.
            urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.S)
            limits = {}
            for m in re.finditer(r'<joint name="([^"]+)" type="(?!fixed)[^"]+"', urdf):
                seg = urdf[m.end():m.end() + 800]
                lim = re.search(r'<limit[^>]*lower="([-\d.eE]+)"[^>]*upper="([-\d.eE]+)"', seg)
                if lim:
                    limits[m.group(1)] = (float(lim.group(1)), float(lim.group(2)))
            missing = [j for j in ARM_JOINTS if j not in limits]
            if missing:
                raise ValueError(f"URDF had no limits for {missing}")
            return limits
        except Exception as exc:  # noqa: BLE001 - any failure means use the fallback
            print(f"[warn] using built-in joint limits ({exc})", file=sys.stderr)
            return dict(FALLBACK_LIMITS)

    # ------------------------------------------------------------------- state

    def joints(self, fresh=True):
        """Measured joint positions by name. Raises if the data is stale."""
        if fresh:
            self._spin(0.1)
            age = time.time() - self._state_t
            if age > STATE_STALE_S:
                raise MotionRefused(f"/joint_states is {age:.2f}s stale")
        return dict(self._state)

    def _peer_publishers(self):
        """Names of the other nodes publishing arm commands right now.

        Existence is not a problem -- astrabot_actuator_sdk and astrabot_mrt both
        hold publishers permanently and are silent most of the time. This is only
        used to say *who* the traffic is from when there is traffic.
        """
        try:
            infos = self.node.get_publishers_info_by_topic(ARM_CMD_TOPIC)
        except Exception:  # noqa: BLE001 - graph queries are best-effort
            return []
        mine = self.node.get_name()
        return sorted({i.node_name for i in infos if i.node_name != mine})

    def _assert_channel_idle(self, window=0.35):
        """Refuse to move if someone else is already streaming arm commands.

        See the ARBITRATION section of the module docstring: the controller takes
        whichever message arrived last, so two live streams means the arm chases
        both. What matters is traffic, not the mere presence of a publisher.
        """
        seen = []
        sub = self.node.create_subscription(
            Float64MultiArray, ARM_CMD_TOPIC, lambda m: seen.append(m),
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        try:
            self._spin(window)
        finally:
            self.node.destroy_subscription(sub)
        if not seen:
            return

        peers = self._peer_publishers()
        who = ", ".join(peers) if peers else "an unidentified publisher"
        hint = ""
        if any("mrt" in p or "mpc" in p for p in peers):
            hint = (" That is the OCS2 stack tracking a reference -- send goals "
                    "via /reference/cmd instead, or stop Astrabot_Mpc.service.")
        elif any("actuator" in p for p in peers):
            hint = " That is probably a canned motion; wait for it to finish."
        raise MotionRefused(
            f"{len(seen)} arm command(s) in the last {window}s from: {who}."
            f" Refusing to fight for the topic.{hint}")

    # -------------------------------------------------------------------- arms

    def move(self, targets, speed=DEFAULT_SPEED, dry_run=False, verify=True):
        """Interpolate the arms to `targets` (dict joint_name -> radians).

        Joints left out of `targets` are held at their measured position -- the
        controller consumes all 14 values at once, so "unspecified" would
        otherwise mean "go to zero".

        speed is the per-joint velocity cap in rad/s; the move takes as long as
        the furthest-travelling joint needs. Returns the achieved pose.
        """
        if not 0 < speed <= MAX_SPEED:
            raise MotionRefused(f"speed {speed} outside (0, {MAX_SPEED}] rad/s")

        targets = _canonical_joints(targets)
        unknown = set(targets) - set(ARM_JOINTS)
        if unknown:
            raise MotionRefused(
                f"not arm joints: {sorted(unknown)}. Expected names like "
                f"'left_arm_4_joint' (the '_joint' suffix is optional).")

        state = self.joints()
        start = [state[j] for j in ARM_JOINTS]
        goal = list(start)
        for j, v in targets.items():
            lo, hi = self.limits[j]
            clamped = min(max(float(v), lo), hi)
            if abs(clamped - float(v)) > 1e-9:
                print(f"[warn] {j}: {v:+.4f} clamped to {clamped:+.4f} "
                      f"(limits {lo:+.3f}..{hi:+.3f})", file=sys.stderr)
            goal[ARM_JOINTS.index(j)] = clamped

        deltas = [g - s for g, s in zip(goal, start)]
        worst = max(abs(d) for d in deltas)
        if worst > MAX_STEP_RAD:
            raise MotionRefused(
                f"largest joint move is {worst:.3f} rad, ceiling is {MAX_STEP_RAD} rad. "
                f"Split it into smaller steps.")

        steps = max(2, int(math.ceil(worst / speed * CMD_RATE_HZ)))
        duration = steps / CMD_RATE_HZ

        moved = [(j, start[i], goal[i]) for i, j in enumerate(ARM_JOINTS)
                 if abs(deltas[i]) > 1e-6]
        print(f"{'DRY RUN: ' if dry_run else ''}{len(moved)} joint(s), "
              f"worst delta {worst:.4f} rad, {duration:.2f}s at <= {speed} rad/s")
        for j, a, b in moved:
            print(f"    {j:20s} {a:+.4f} -> {b:+.4f}  ({b - a:+.4f} rad)")
        if not moved:
            return state
        if dry_run:
            return state

        self._assert_channel_idle()

        period = 1.0 / CMD_RATE_HZ
        msg = Float64MultiArray()
        next_t = time.time()
        for s in _raised_cosine(steps):
            msg.data = [a + s * d for a, d in zip(start, deltas)]
            self._arm_pub.publish(msg)
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            rclpy.spin_once(self.node, timeout_sec=0.0)

        # Re-assert the endpoint: the drives need a moment to settle, and a
        # single dropped final message would leave the arm short of target.
        for _ in range(5):
            msg.data = list(goal)
            self._arm_pub.publish(msg)
            self._spin(0.05)

        self._spin(0.3)
        reached = self.joints()
        if verify:
            off = [(j, reached[j], goal[ARM_JOINTS.index(j)]) for j, _, _ in moved
                   if abs(reached[j] - goal[ARM_JOINTS.index(j)]) > ARRIVAL_TOL_RAD]
            if off:
                print(f"[warn] {len(off)} joint(s) did not reach target within "
                      f"{ARRIVAL_TOL_RAD} rad:", file=sys.stderr)
                for j, got, want in off:
                    print(f"    {j:20s} want {want:+.4f}  got {got:+.4f}", file=sys.stderr)
        return reached

    def move_through(self, waypoints, speed=DEFAULT_SPEED, guard=None, verify=True):
        """Stream ONE continuous ramp through a whole joint-space polyline.

        move() ramps to zero velocity at every call and then settles ~0.35 s, so
        an N-waypoint path pays N accel/decel cycles plus N settles -- visible as
        a stutter on descent. Here the entire polyline gets a single raised-cosine
        time scaling over its total joint travel, so intermediate waypoints are
        passed at speed and only the endpoints are at rest.

        guard(measured) -> str | None, called at ~20 Hz with the measured joint
        dict. A returned string aborts the stream and is raised as MotionRefused.
        This robot reports `effort` as .nan, so an in-loop geometric guard is the
        only backstop there is -- do not drop it to save a few lines.

        Per-segment travel still has to be inside MAX_STEP_RAD (the planner emits
        small steps); the *total* is deliberately unbounded, that being the point.
        """
        if not 0 < speed <= MAX_SPEED:
            raise MotionRefused(f"speed {speed} outside (0, {MAX_SPEED}] rad/s")
        wps = [_canonical_joints(w) for w in waypoints]
        if not wps:
            raise MotionRefused("move_through: no waypoints")
        for w in wps:
            unknown = set(w) - set(ARM_JOINTS)
            if unknown:
                raise MotionRefused(f"not arm joints: {sorted(unknown)}")

        state = self.joints()
        pts = [[state[j] for j in ARM_JOINTS]]
        for w in wps:
            p = list(pts[-1])
            for j, v in w.items():
                lo, hi = self.limits[j]
                p[ARM_JOINTS.index(j)] = min(max(float(v), lo), hi)
            pts.append(p)

        seg = [max(abs(b - a) for a, b in zip(p, q)) for p, q in zip(pts, pts[1:])]
        if max(seg) > MAX_STEP_RAD:
            raise MotionRefused(
                f"segment {seg.index(max(seg))} moves {max(seg):.3f} rad, "
                f"ceiling is {MAX_STEP_RAD} rad")
        total = sum(seg)
        if total < 1e-6:
            return state
        cum = [0.0]
        for s in seg:
            cum.append(cum[-1] + s)

        steps = max(2, int(math.ceil(total / speed * CMD_RATE_HZ)))
        print(f"streaming {len(pts) - 1} segment(s), {total:.4f} rad total, "
              f"{steps / CMD_RATE_HZ:.2f}s at <= {speed} rad/s")

        self._assert_channel_idle()
        period = 1.0 / CMD_RATE_HZ
        msg = Float64MultiArray()
        next_t = time.time()
        k = 0
        for i, u in enumerate(_raised_cosine(steps)):
            a = u * total
            while k < len(seg) - 1 and a > cum[k + 1]:
                k += 1
            span = seg[k] or 1.0
            t = min(1.0, max(0.0, (a - cum[k]) / span))
            msg.data = [p + t * (q - p) for p, q in zip(pts[k], pts[k + 1])]
            self._arm_pub.publish(msg)
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if guard is not None and i % 5 == 4 and self._state is not None:
                why = guard(dict(self._state))
                if why:
                    # Hold where we are: stopping the stream on a position
                    # controller means the last setpoint stands.
                    for _ in range(3):
                        self._arm_pub.publish(msg)
                        self._spin(0.02)
                    raise MotionRefused(f"aborted mid-stream: {why}")

        for _ in range(5):
            msg.data = list(pts[-1])
            self._arm_pub.publish(msg)
            self._spin(0.05)
        self._spin(0.3)
        reached = self.joints()
        if verify:
            off = [(j, reached[j], pts[-1][i]) for i, j in enumerate(ARM_JOINTS)
                   if abs(reached[j] - pts[-1][i]) > ARRIVAL_TOL_RAD]
            if off:
                print(f"[warn] {len(off)} joint(s) short of target:", file=sys.stderr)
                for j, got, want in off:
                    print(f"    {j:20s} want {want:+.4f}  got {got:+.4f}", file=sys.stderr)
        return reached

    def home(self, which="both", speed=DEFAULT_SPEED, dry_run=False):
        """Move to the all-zeros pose, which is this robot's home.

        Every canned motion in astrabot_motion_list.yaml begins and ends there,
        so it is the vendor's own reference pose.
        """
        sides = {"both": ARM_JOINTS,
                 "left": [j for j in ARM_JOINTS if j.startswith("left")],
                 "right": [j for j in ARM_JOINTS if j.startswith("right")]}
        if which not in sides:
            raise MotionRefused(f"which must be one of {sorted(sides)}")
        joints = sides[which]
        state = self.joints()
        segments = max(1, math.ceil(max(abs(state[j]) for j in joints) / MAX_STEP_RAD))
        if dry_run or segments == 1:
            return self.move({j: 0.0 for j in joints}, speed=speed, dry_run=dry_run)
        waypoints = [
            {j: state[j] * (segments - i) / segments for j in joints}
            for i in range(1, segments + 1)
        ]
        return self.move_through(waypoints, speed=speed)

    # -------------------------------------------------------------------- head

    def look_at(self, pitch=None, yaw=None, speed=DEFAULT_SPEED, dry_run=False):
        """Point the head (and the ZED 2i mounted on it).

        Requires astrabot_neck_forward_controller to be active; it is loaded
        `--inactive` by /home/astrabot/config/astrabot_xr1_evt2_arm_head_safe.launch.py
        so that a restart cannot make the head snap. Activate with:
            ros2 control set_controller_state astrabot_neck_forward_controller active
        """
        # The arm-only bring-up loads a URDF where the head joints are `fixed`, so
        # _load_limits() skips them and there is nothing to clamp against. Say so
        # rather than dying on a bare KeyError.
        unmodelled = [j for j in NECK_JOINTS if j not in self.limits]
        if unmodelled:
            raise MotionRefused(
                f"no travel limits for {unmodelled} in the active robot_description -- "
                "the arm-only bring-up models the head as fixed. Restart the Controller "
                "on astrabot_xr1_evt2_arm_head_safe.launch.py before pointing the head.")

        state = self.joints()
        start = [state[j] for j in NECK_JOINTS]
        goal = list(start)
        for idx, val in ((0, pitch), (1, yaw)):
            if val is None:
                continue
            lo, hi = self.limits[NECK_JOINTS[idx]]
            goal[idx] = min(max(float(val), lo), hi)
            # Head travel is only +-40 deg. Silently clamping a 1.5 rad request to
            # 0.698 would let a caller believe it looked much further than it did.
            if abs(goal[idx] - float(val)) > 1e-9:
                print(f"[warn] {NECK_JOINTS[idx]} target {float(val):+.3f} clamped to "
                      f"{goal[idx]:+.3f} (limits {lo:+.3f}..{hi:+.3f}); head travel is "
                      f"+-40 deg, turn the base for anything wider", file=sys.stderr)

        deltas = [g - s for g, s in zip(goal, start)]
        worst = max(abs(d) for d in deltas)
        print(f"{'DRY RUN: ' if dry_run else ''}head pitch {start[0]:+.3f}->{goal[0]:+.3f}, "
              f"yaw {start[1]:+.3f}->{goal[1]:+.3f}")
        if worst < 1e-6 or dry_run:
            return state

        steps = max(2, int(math.ceil(worst / speed * CMD_RATE_HZ)))
        msg = Float64MultiArray()
        for s in _raised_cosine(steps):
            msg.data = [a + s * d for a, d in zip(start, deltas)]
            self._neck_pub.publish(msg)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._spin(0.3)
        return self.joints()

    # ---------------------------------------------------------------- grippers

    def gripper(self, side, value):
        """Command a gripper. `value` is passed straight through as a Float64.

        UNVERIFIED SCALE. The hardware is a NiMotion 4th-gen single-finger
        gripper (gripper_hw_ver: 5, force: 100) and the URDF models the finger
        as prismatic 0..0.045 m, but the units this topic expects -- metres,
        normalised 0..1, or percent -- have not been confirmed against the
        hardware, and /astrabot/gripper_{left,right}_state read all-zeros while
        idle. Watch the hardware the first time you call this, and start in the
        middle of whatever range you believe it to be.
        """
        if side not in self._grip_pub:
            raise MotionRefused(f"side must be 'left' or 'right', not {side!r}")
        print(f"gripper {side}: publishing {value} to {GRIPPER_TOPIC[side]} "
              f"(scale unverified)")
        self._grip_pub[side].publish(Float64(data=float(value)))
        self._spin(0.2)

    # -------------------------------------------------------------------- lift

    def set_height(self, metres):
        """Command the body lift height.

        UNVERIFIED RANGE. astrabot_actuator_sdk drives the hip/knee/ankle
        actuators over its own CAN link from this single scalar, and its
        internal log format (`testLift: height %.04f`) suggests metres, but no
        travel limits for it are published anywhere in the ROS graph or in
        astrabot_actuator_sdk's config. Do not guess a value: read the current
        one off the hardware first, and move in small increments.
        """
        print(f"lift: publishing {metres} to {HEIGHT_TOPIC} (range unverified)")
        self._height_pub.publish(Float64(data=float(metres)))
        self._spin(0.2)

    # ------------------------------------------------------------------ teardown

    def close(self):
        self.node.destroy_node()
        if self._owns_ros:
            rclpy.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", action="store_true", help="print measured joint positions")
    ap.add_argument("--move", nargs="+", metavar="JOINT=RAD",
                    help="move named joints, e.g. --move left_arm_2=0.15")
    ap.add_argument("--home", choices=("left", "right", "both"), help="go to the zero pose")
    ap.add_argument("--look-at", nargs=2, type=float, metavar=("PITCH", "YAW"),
                    help="point the head (radians); needs the neck controller active")
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                    help=f"per-joint velocity cap in rad/s (default {DEFAULT_SPEED})")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the target vector instead of publishing it")
    args = ap.parse_args()

    if not any((args.state, args.move, args.home, args.look_at)):
        ap.error("nothing to do; pass --state, --move, --home or --look-at")

    try:
        with Robot() as bot:
            if args.state:
                measured = bot.joints(fresh=False)
                for j in NECK_JOINTS + ARM_JOINTS:
                    lo, hi = bot.limits[j]
                    pos = measured.get(j, float("nan"))
                    # A joint can sit outside its own declared range: the encoder
                    # reports where the mechanism actually is, and the URDF is
                    # just someone's number. right_arm_4_joint does exactly this
                    # (+0.0513 against an upper bound of 0.000) because the EVT2
                    # URDF truncated a small positive bound to zero -- the older
                    # astrabot_xr1_description says +0.13 for the same joint.
                    # Flag it rather than printing it as if it were fine: every
                    # target for this joint is being clamped to a bound the
                    # hardware has already demonstrably passed.
                    flag = ""
                    if pos == pos:  # not NaN
                        if pos > hi:
                            flag = f"  <-- ABOVE upper by {pos - hi:+.4f}"
                        elif pos < lo:
                            flag = f"  <-- BELOW lower by {pos - lo:+.4f}"
                        elif min(hi - pos, pos - lo) < 0.02:
                            flag = f"  <-- within {min(hi - pos, pos - lo):.4f} of a limit"
                    print(f"{j:20s} {pos:+.4f}   [{lo:+.3f}, {hi:+.3f}]{flag}")
            if args.move:
                targets = {}
                for item in args.move:
                    name, _, val = item.partition("=")
                    if not val:
                        ap.error(f"--move takes JOINT=RAD pairs, got {item!r}")
                    targets[name] = float(val)
                bot.move(targets, speed=args.speed, dry_run=args.dry_run)
            if args.home:
                bot.home(args.home, speed=args.speed, dry_run=args.dry_run)
            if args.look_at:
                bot.look_at(pitch=args.look_at[0], yaw=args.look_at[1],
                            speed=args.speed, dry_run=args.dry_run)
    except MotionRefused as exc:
        print(f"[refused] {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
