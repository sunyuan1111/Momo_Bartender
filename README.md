# Physical Agent

This repository provides a small, extensible control layer for a 6-axis arm plus gripper
driven by 7 Feetech `sts3215` servos through `LeRobot`'s `FeetechMotorsBus`.

Current scope:

- `sts3215` step mode (`Operating_Mode = 3`)
- power-on pose recorded as the software zero point
- 7-servo default configuration
- `joint_2` and `joint_3` configured with a `14:1` reduction ratio
- basic single-joint and multi-joint motion
- gripper motion through a dedicated API and CLI command

The code intentionally stays small so we can keep iterating once the URDF and higher-level
kinematics are ready.

## Layout

- `configs/arm7_sts3215.example.json`: default servo layout and serial settings
- `models/arm2/`: ROS-style URDF package for the exported arm model
- `models/arm2/config/joint_limits.json`: per-joint limit data for the `arm2` URDF
- `scripts/update_urdf_limits.py`: utility to write joint limits back into a URDF
- `src/physical_agent/config.py`: typed config loader
- `src/physical_agent/controller.py`: bus-backed arm controller
- `src/physical_agent/cli.py`: simple command-line entrypoint

## Install

```bash
./scripts/setup_momo_env.sh
conda activate momo
```

This setup uses Python 3.12 and installs the minimum hardware-side packages needed for the
current controller code. `LeRobot` is installed without its full ML stack, then the Feetech bus
dependencies are installed explicitly.

## Quick Start

1. Copy `configs/arm7_sts3215.example.json` and adjust the serial port, motor IDs, and axis
   directions for your arm.
2. Initialize the bus and put every servo into step mode:

```bash
physical-agent init --config configs/arm7_sts3215.example.json
```

3. Move one or more joints in degrees relative to the power-on pose:

```bash
physical-agent move-joints \
  --config configs/arm7_sts3215.example.json \
  --position joint_1=10 \
  --position joint_2=-5
```

4. Control the gripper separately:

```bash
physical-agent move-gripper \
  --config configs/arm7_sts3215.example.json \
  --position 15
```

5. Read the current raw register state plus the controller's tracked targets:

```bash
physical-agent state --config configs/arm7_sts3215.example.json
```

## Motion Model

This project treats the power-on pose as the logical zero pose. Because `sts3215` step mode is
incremental, the controller tracks commanded joint targets in software and sends the delta between
the current target and the next target to `Goal_Position`.

For a commanded joint delta:

```text
motor_delta_deg = joint_delta_deg * gear_ratio * direction
motor_delta_raw = round(motor_delta_deg / 360 * 4096)
```

The controller writes that signed raw delta with:

```python
bus.write("Goal_Position", joint_name, int(goal_raw), normalize=False)
```

## Assumptions

- Default motor IDs are `1..7`.
- All 7 motors are `sts3215`.
- `joint_2` and `joint_3` use a `14:1` ratio.
- The gripper is servo `7`.
- Step mode is configured at startup.
- The current code focuses on low-level motion only; URDF alignment and kinematics can be added
  later without restructuring the bus layer.

## URDF Limits

Store model-specific joint limits next to the model under `models/<robot>/config/`, then apply
them with the repository-level helper script:

```bash
python3 scripts/update_urdf_limits.py \
  models/arm2/urdf/arm2.urdf \
  --limits-file models/arm2/config/joint_limits.json
```

If you want to scaffold a new limits file from an existing URDF:

```bash
python3 scripts/update_urdf_limits.py \
  models/arm2/urdf/arm2.urdf \
  --write-template models/arm2/config/joint_limits.json
```
