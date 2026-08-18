# Grippers --- UFactory G2 over Modbus RTU

Current state of the subsystem. Every number here was measured on this machine
(read-only Modbus `0x03` scan → visual confirmation of motion → end-to-end
closed-loop check → quantified frame loss), 2026-08-07. The mistakes made on the
way to it are in [`../operations/pitfalls.md`](../operations/pitfalls.md) §2, §3,
§4, §16, §21, §39 --- read those before debugging "the gripper is broken".

## Wiring

| | Left | Right |
|---|---|---|
| Device node | `/dev/ttyAMA5` | `/dev/ttyUSB0` |
| Physical layer | Thor native PL011 UART `810c510000.serial` | CP2102N USB serial, USB `1-3.1` |
| Stable identifier | `/sys/class/tty/ttyAMA5` is fixed | `ID_SERIAL_SHORT=d60f7389ced5ef118820724b49d2c684` |
| Modbus slave id | 8 | 8 |
| Baud | 2000000 | 2000000 |
| Frame loss | ~1--2 %, retried to 0 (see below) | 0 % |

They are two distinct physical devices, not two names for one port --- the first
scan read different `Fn702` values (548 vs 560).

Thor's native UART is PL011 → `/dev/ttyAMA*`. There is no `/dev/ttyTHS*` on this
machine.

- `/dev/ttyAMA4`: silent on 6 bauds × 5 slave ids. Unknown. Do not assume it is
  free and attach something to it.
- `/dev/ttyAMA10`: **do not touch.** Neck SM45BL servos, held exclusively by
  `ros2_control_node`'s `AstraNeckHW`. Opening it takes head control away. The
  device name is compiled into the binary --- it is in no config file.

## Registers

From `G2Gripper` in `g2_gripper_node.py`:

| Name | Address | Manual | Len | Use |
|---|---|---|---|---|
| `REG_ENABLE` | `0x0100` | `Fn100` | 1 | write 1 to enable. **Zero at power-on** |
| `REG_FNC_BASE` | `0x0C00` | `FnC00..04` | 5 | position command block |
| `REG_FDBK_POS` | `0x0702` | `Fn702..03` | 2 | position feedback, `(reg0 << 16) \| reg1` |

Command block (FC `0x10`, 5 registers): `[1, speed_cmd, force_cmd, pos_hi, pos_lo]`,
where the position is millimetres of opening split across two words.

```
open01  = 1 - close01                    # teleop convention: 0 = fully open, 1 = fully closed
pos_mm  = round(open01 * 840)            # max_position_mm = 840
```

CRC16 polynomial `0xA001` (standard Modbus). Inter-frame gap is 3.5 characters
≈ 20 µs at 2 Mbaud.

Closed-loop check (command → Modbus → motion → feedback), both hands:

| `close01` | expected | left | right |
|---|---|---|---|
| 0.65 | 294 | 293 | 291 |
| 0.00 | 840 | 838 | 840 |
| 0.35 | 546 | 545 | 544 |

Errors are at encoder-noise level.

## Driver

`g2_gripper_pc` in `/home/astrabot/gripper_ws` (from a user-supplied
`xarm_gripper.tar` --- the filename is wrong, the contents are a UFactory G2
RS485 driver). Rebuilt for Jazzy / python3.12:

```bash
cd /home/astrabot/gripper_ws && colcon build --packages-select g2_gripper_pc
```

Local config changes in `config/g2_gripper_config.yaml`: the two port names
(upstream ships `ttyUSB3`/`ttyUSB2`, which do not exist here --- the original
machine had two USB dongles, this one has one dongle plus one native UART),
`modbus_retries: 2`, and the bus health counters. The bundled
`99-usb-grippers-g2.rules` matches neither of this machine's serial numbers ---
**do not install it.**

Defaults: `speed_cmd 3000`, `force_cmd 50`, `write_min_period_ms 100`,
`publish_frequency 20.0`. **`force_cmd` has never been calibrated**; 30 and 50
are the only two values anyone has qualitative experience with.

`py/xr1.py bringup` starts it. It is **not** a systemd unit, so it does not come
back after a reboot.

## Topics

| | Topic | Type |
|---|---|---|
| command | `/rm_{left,right}/rm_driver/teleop_gripper_float` | `std_msgs/Float64`, 0 = open, 1 = closed |
| state | `/qg_robot/gripper_{left,right}_state` | `std_msgs/UInt32MultiArray`, `data[0]` = mm |

The command topic names are **identical** to the vendor SDK's, so
`g2_gripper_pc` and the SDK's gripper logic must never run at once. The vendor
config was pointed at inert topic names for exactly this reason --- but its
`gripper_list` must stay non-empty or `ros2_control_node` aborts on every boot
(pitfalls §2).

Ignore `/astrabot/gripper_{left,right}_state`: that is the vendor's wrong SDK and
it publishes `[0,0,0,0]` forever. Both sets exist in domain 12, which makes them
easy to confuse.

Only `data[0]` is real. `running`, `temp` and `error` are hard-coded zeros in
`g2_gripper_node.py`, not sensor readings.

## The only "did I grab it" signal

There is no force or torque feedback anywhere on this robot (`effort` is `.nan`
on all 16 joints) and `force_cmd` is open loop. So:

> Compare the commanded opening against the actual `pos_mm`. Stuck **wider** than
> commanded ⇒ something is between the fingers.

Read it **static**, and confirm with a lift. On 2026-08-18: 149 closed on the
object, 14 closed on air, still 148 after lifting. Sampling at the instant the
command is issued records the home pose (839, fully open) as a success.

## Frame loss on `ttyAMA5`, and the retry

`ttyAMA5` silently dropped ~1--2 % of Modbus transactions while `ttyUSB0` in the
same run dropped none. Raising the timeout does nothing --- 0.05 / 0.15 / 0.30 s
gave 0.3 / 1.3 / 1.0 %, flat within noise, while a successful reply takes 0.8 ms
(worst 4.2 ms). Every failure returned **zero bytes**: not a short frame, not a
CRC error. The whole frame vanishes in the PL011's half-duplex turnaround.
Waiting longer cannot recover it; asking again can.

`G2ModbusClient._txrx()` retries twice with a 2 ms gap (≫ the 20 µs inter-frame
time, so a late half-frame drains before the retry). This is safe for every
function code the driver uses: `0x03` is a pure read, and `0x06`/`0x10` write
**idempotent absolute values** (enable = 1, or an absolute target position), so a
resend after a lost reply cannot accumulate. Result: 3902 transactions, 79
retries (2.02 %), **0 final failures**; user-visible warnings went from 38 in
30 s to 0 in 110 s.

Retrying alone would *hide* bus degradation --- 1 % and 30 % loss look equally
quiet. So `tx_total` / `tx_retried` / `tx_failed` ride the existing 15 s
diagnostic tick:

```
[bus] left /dev/ttyAMA5 tx=3902 retried=79 (2.02%) failed=0 (0.00%)
```

**`retried%` is the bus health metric.** ~2 % left, 0 % right is normal. If it
climbs, the wiring, shielding or ground is degrading --- go look at the hardware,
do not touch the timeout.

## Safety

- Enabling (`Fn100 = 1`) does **not** trigger a homing sweep on these two units
  (measured: position unchanged), contrary to what the G2 manual implies. Clear
  the area anyway.
- `Fn100` is latched in the drive across power cycles; it stays 1 after the node
  exits.
- First motion of a session goes to `0.0` (fully open), which cannot grab
  anything.
- Discovery-phase work uses read-only `0x03` only.

## Open

- No force calibration (`force_cmd` → newtons is unknown).
- No autostart unit; needs an entry in `/opt/ros/start_up/auto_start_script/`
  including `ROS_DOMAIN_ID=12`.
- No udev rule pinning `/dev/ttyUSB0` by `ID_SERIAL_SHORT`, so the number can
  drift if another USB serial device is plugged in.
- `/dev/ttyAMA4` is unidentified.
- The local driver changes (ports, retries, counters) exist only on this machine
  and should go back upstream.
