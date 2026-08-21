# Runbook

Everything here moves hardware or takes a device away from another session.
Read [`status.md`](./status.md) first.

## 0. Every session

```bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
python3 py/xr1.py pose        # also proves /joint_states is alive
bin/tf-frames                 # 52 total, 6 zed_*; anything else is a known failure
python3 py/xr1.py bringup     # after a reboot: the G2 gripper driver is not a systemd unit
```

Head pose is a precondition, not a preference: `pitch = +40°` (reads 39) and
`yaw = 0`. At any other pitch the ZED and the arm's workspace do not overlap; at
40° of yaw you get a half-metre localisation error.

Sudo needs an askpass helper (there is no TTY):

```bash
printf '#!/bin/sh\necho 1\n' > /tmp/askpass.sh && chmod +x /tmp/askpass.sh
export SUDO_ASKPASS=/tmp/askpass.sh    # then: sudo -A <cmd>
```

## 1. Observe and plan

```bash
bin/xr1 observe        # -> data/vista_runs/yellow-block-harness/latest.json
bin/xr1 bundle         # validate and print the unified observation contract
bin/xr1 validate-proposal --proposal examples/pick_place_proposal.json
bin/xr1 plan           # immutable Release attempt receipt
bin/xr1 plan --proposal examples/grasp_proposal.json
bin/xr1 plan --proposal examples/grasp_proposal.json --latest SAVED_LATEST_JSON
bin/xr1 replay --proposal examples/pick_place_proposal.json \
  --events examples/task_events.jsonl
```

`bundle`, `validate-proposal`, `plan` and `replay` move nothing. Before believing
plan numbers, **open the actual image**.
An aggregate value can only confirm a hypothesis you already got right --- one
session read "yellow pixel count" off nine wrist frames, concluded the table was
empty, and then scanned the wrong axis for twenty minutes.

Production `plan` writes one immutable result under
`data/attempts/attempt_<frame>_<key>/`. Repeating the same request reads its
saved attempt; it never invokes the planner again. The command returns only a
short receipt containing `attempt_path` and `plan_path`; full diagnostics stay
in the attempt directory. The execution adapter accepts `--attempt DIR`, never
an arbitrary plan file. The key binds the snapshot,
joint state, image-time TF, RGB/depth/intrinsics, URDF, calibration inputs,
proposal, planner revision, safety policy and optional MoveIt validator/SRDF. After
the robot or scene changes, observe again; the new state necessarily gets a new
key and a new plan.

From a workstation use `bin/xr1 --host astrabot@192.168.123.102 COMMAND ...`.
It runs one SSH command without a PTY and exits; do not maintain an interactive
SSH shell or poll it with fixed 30-second waits.

### Bounded near-target alignment

With a 3×3 calibration measured at the current pose family:

```bash
bin/xr1 servo-loop --calibration /tmp/servo-calibration.json
bin/xr1 servo-loop --calibration /tmp/servo-calibration.json --go
```

The first command observes live state and dry-runs the next approved microstep.
The second explicitly permits motion. Each iteration is one new ZED observation,
one Rust-gated microstep, another distinct observation, then predicted/actual
reconciliation. Defaults are six steps and 180 s; concurrent loops are refused.
It uses running publishers as-is and never starts, stops or restarts services.

Do not add `--require-d405`, `--require-tactile` or
`--require-force-feedback` to make a result look safer. D405 and tactile become
healthy only for a fresh validated real sample; arm force feedback remains
unavailable. Convergence authorizes neither gripper close nor a claim of grasp
success; those remain separate actions and evidence.

### D405 and pressure-assisted final grasp

Copy the three files under `examples/*.example.json`, replace every placeholder
with measured values, and keep the originals as a reminder that defaults are
not calibration. First verify each read-only boundary:

On the robot, production `/usr/bin/python3` must import `pyrealsense2`; the
validated user-local binding is `2.58.3.10794` for CPython 3.12 aarch64. Do not
switch this boundary to the deploy venv, because the same process also needs the
ROS 2 Python packages.

```bash
bin/xr1 d405-observe
bin/xr1 tactile-observe --config /tmp/tactile.json
bin/xr1 tactile-assess --mode baseline \
  --config /tmp/tactile.json --calibration /tmp/tactile-calibration.json
```

Every accepted D405 observation atomically retains first/middle/last RGB-depth
keyframes in `data/sensors/d405/observations/<frame-id>/keyframes.json`. They are
occlusion memory for inspection, not standalone motion authorization.

Use repeated `tactile-observe` samples with the jaws open, then gentle manual
pressure on each named patch, to determine baseline, polarity, noise, contact,
balance and a human-approved pressure ceiling. Do not derive a safety ceiling
from USB enumeration or from one arbitrary sample.

Use a D405 Jacobian measured at the current pose family for final alignment:

```bash
bin/xr1 servo-loop --calibration /tmp/d405-jacobian.json \
  --d405-target /tmp/d405-grasp-target.json
# Inspect the fresh image and dry-run report before the separate --go action.
```

Once the target is inside tolerance and the gripper is already open, dry-run
the contact loop. A real run performs only one jaw increment per fresh pressure
sample and rechecks D405 before any further close:

```bash
bin/xr1 grasp-loop --tactile-config /tmp/tactile.json \
  --tactile-calibration /tmp/tactile-calibration.json \
  --d405-target /tmp/d405-grasp-target.json
# Repeat with --go only after the dry run passes.
```

Balanced two-pad contact ends in `contact`; one-sided contact ends in
`imbalanced`; the pressure ceiling permits one opening increment and then
stops. The loop has a 360 s default overall deadline in addition to its 20-step
ceiling. `lift_authorized` remains false: lifting is a separate arm action, after
which `tactile-assess --mode retention --held HELD --observation AFTER_LIFT`
checks that both pads retained pressure without an excessive drop.

## 2. Run an experiment

```bash
bin/xr1 begin --purpose "..."
bin/xr1 note --section observation --text "..."
bin/xr1 grip --side right --state close
bin/xr1 end --status SUCCESS
```

One action, then observe again. Write the prediction as a number *before*
acting, and reconcile afterwards --- if prediction and reality disagree, the
model is wrong, and the model is what gets fixed. Not the gate.

The full cycle, fixed by the operator on 08-14 and unchanged since: observe →
read the visual state → list what is executable → **reason (this step is the
agent, never code)** → check the history → write the prediction → execute
**one** action → observe again → the new frame becomes the current state →
compare prediction with reality → correct the understanding → next round.
The one-shot pipeline that used to do all of it in one command compressed every
one of those steps into a single exit code, and 30 runs stayed stuck on the same
code path without anyone noticing.

**Grasp success criterion**: a *static* gripper reading plus a lift. On 08-18,
149 closed on the object versus 14 closed on air, and still 148 after lifting.
"The gripper closed" proves nothing, and reading the position at the instant the
trigger is pulled records the home pose (839, fully open) as a success.

## 3. Recording (external camera on the Mac)

```bash
python3 py/xr1_cam.py doctor
python3 py/xr1_cam.py start <clip>
python3 py/xr1_cam.py stop
python3 py/xr1_cam.py pull <clip> --dest DIR
```

The recorder is at `apple@192.168.123.138` (`ssh -i ~/.ssh/id_xr1rec`), clips in
`/Users/apple/xr1rec/clips/*.mov`, 1080p30 ≈ 2 MB/s.

It is **exclusive**. `stop` from another session silently voids whatever
experiment was running, so `doctor` first --- `state=recording` means someone
else is live. When ordering evidence, trust `rec_confirm_ms`, not filenames.

⚠️ The last confirmed working recording was before 2026-08-11 and its clip is
gone with `_attic_20260811/`. Treat "the recording link works" as a *historical*
claim until you have run it once with `--require-video`.

## 4. VR teleoperation

⚠️ Once the Quest takes control the graph **drives the arms**. Keep a hand on the
e-stop.

Three things must hold simultaneously, and any one missing looks identical
("connected but no image / no motion"):

1. `Astrabot_Data_Collection.service` is **stopped**. It crash-loops every 5 s and
   grabs the token server, the cameras and dora.
2. LiveKit media (:7880) and token (:5000) both active, and a POST to
   `http://192.168.123.102:5000/token` returns 200. The daemon connects to `.102`,
   not to `127.0.0.1`, so verify that address specifically. (000/refused = the
   token server is down; 405 to a GET still means alive.)
3. The Quest's identity differs from the robot's `ASTRABOT-4`, or whoever joins
   second kicks the other out (`Room disconnected: DuplicateIdentity`).

```bash
sudo -A systemctl stop Astrabot_Data_Collection.service
sudo -A systemctl start Astrabot_LiveKit.service Astrabot_LiveKit_Srv.service
curl -s -o /dev/null -w '%{http_code}\n' -X POST \
  -H 'Content-Type: application/json' -d '{}' http://192.168.123.102:5000/token

cd /home/astrabot/deploy
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export VIRTUAL_ENV=/home/astrabot/deploy/.venv PATH=/home/astrabot/deploy/.venv/bin:$PATH
.venv/bin/dora run .astra/astrabot_data_collection_xr1_evt2.yml
```

Three preconditions for that graph, each of which fails with an error pointing
somewhere else:

- **venv bin first in `PATH`.** The yml writes `path: python` --- a bare name,
  resolved from `PATH`. Without it every node raises
  `ModuleNotFoundError: No module named 'hardware'`, which reads like a missing
  package and is actually the wrong interpreter.
- **`ROS_DOMAIN_ID=12`**, because `control_astrabot` uses rclpy and TF.
- **ZED released.** `Astrabot_ZED.service` is `Restart=always` with
  `RestartUSec=100ms`, so it wins any race --- an explicit `systemctl stop` is the
  only way. Otherwise the next start fails with
  `Failed to open ZED camera: CAMERA NOT DETECTED` while `lsusb` still shows it.
  **Better: delete the optional `eye_zed` node from the yml** and skip this
  entirely.

There are two different config files and they are not interchangeable:
`/home/astrabot/config/data_collection_xr1_evt2.yaml` (3.2 KB, used by
`astra run`) and `deploy/.astra/astrabot_data_collection_xr1_evt2.yml` (6.6 KB,
10 nodes, used by `dora run`).

**Tearing the graph down**: kill by *PID range*, not by name. When the `dora run`
parent takes SIGTERM its ~10 children orphan to `ppid=1` and keep running under
ten different module names, so a name-based `pgrep` list always misses some ---
and a leftover `control_astrabot` will make the next instance's
`failed to connect TF` look like a port conflict. Children of one `dora run` have
consecutive PIDs. Never `pkill -f dora`: it matches your own shell.

**Success criterion**: fingertip FK leaving the zero pose. `elapsed` not
incrementing does not mean it is working, and `use_byte: false` plus a dead G2
driver are *necessary but not sufficient* --- on 08-12 both were green and the
arms still did not move, because the Quest app was sending the literal string
`hello world`.

## 5. Concurrency

`pgrep -af` before any `pkill`. Restarting a service, claiming a serial port and
taking the recorder are all outward-facing actions on a shared machine. If you
leave the arm somewhere unusual, say so in `status.md`.
