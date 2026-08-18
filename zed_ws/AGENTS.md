# Thor workspace agent instructions

- For any request requiring the robot's current visual state, ZED snapshot, aligned depth, camera intrinsics, joint state, or image-time TF, use the `thor-observe` skill first.
- Canonical skill: `/home/astrabot/.codex/skills/thor-observe/SKILL.md`.
- Read-only implementation: `/home/astrabot/workspace/zed_ws/scripts/vista_observe.py`.
- Never open the ZED directly with `pyzed`; `Astrabot_ZED.service` owns the camera.
- The observe skill must not move the robot, publish ROS commands, or restart services without explicit user authorization.
