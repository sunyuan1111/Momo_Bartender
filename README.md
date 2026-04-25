# MOMO BARTENDER

This repository for #AttraX_Sping_Hackthon

This repository provides a small, extensible control layer for a 6-axis arm plus gripper
driven by 7 Feetech `sts3215` servos through `LeRobot`'s `FeetechMotorsBus`.

Current scope:

- `sts3215` position-servo mode (`Operating_Mode = 0`)
- per-joint zero references through `zero_position_raw`
- 7-servo default configuration
- `joint_2` and `joint_3` configured with a `14:1` reduction ratio
- basic single-joint and multi-joint motion
- gripper motion through a dedicated API and CLI command
- URDF-based Cartesian `x/y/z` control for the 6 arm joints

The code intentionally stays small so we can keep iterating once the URDF and higher-level
kinematics are ready.

## Layout

- `configs/arm7_sts3215.example.json`: default servo layout and serial settings
- `models/arm2/`: ROS-style URDF package for the exported arm model
- `models/arm2/config/joint_limits.json`: per-joint limit data for the `arm2` URDF
- `scripts/update_urdf_limits.py`: utility to write joint limits back into a URDF
- `src/physical_agent/config.py`: typed config loader
- `src/physical_agent/controller.py`: bus-backed arm controller
- `src/physical_agent/kinematics.py`: minimal URDF FK/IK for Cartesian control
- `src/physical_agent/cli.py`: simple command-line entrypoint

## Install

```bash
./scripts/setup_momo_env.sh
conda activate momo
pip install -e ".[gui]"
```

This setup uses Python 3.12 and installs the minimum hardware-side packages needed for the
current controller code. `LeRobot` is installed without its full ML stack, then the Feetech bus
dependencies are installed explicitly. The GUI is optional and uses `PyQt5`.

## Quick Start

1. Copy `configs/arm7_sts3215.example.json` and adjust the serial port, motor IDs, and axis
   directions for your arm. By default, every joint uses `zero_position_raw = 2048`.
2. Initialize the bus and put every servo into position mode:

```bash
physical-agent init --config configs/arm7_sts3215.example.json
```

3. Move one or more joints in degrees relative to each joint's configured zero reference:

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

5. Read the current raw state, present joint angles, and tracked targets:

```bash
physical-agent state --config configs/arm7_sts3215.example.json
```

6. Use the `arm2` URDF for Cartesian control:

```bash
physical-agent cartesian-state --config configs/arm2_sts3215.example.json
physical-agent solve-cartesian --config configs/arm2_sts3215.example.json --x 0.10 --y 0.05 --z 0.18
physical-agent move-cartesian --config configs/arm2_sts3215.example.json --x 0.10 --y 0.05 --z 0.18
```

## Quick Move GUI

The repository now includes a stripped-down GUI that keeps only the `Quick Move` workflow and
removes features that are not supported by the current backend.

- single-window Quick Move interface
- connect / disconnect
- Home
- per-joint step jog
- Cartesian `X/Y/Z` jog in base frame
- live joint and TCP state refresh

Deliberately removed from the GUI because the current controller does not support them cleanly:

- orientation `Rx/Ry/Rz` jog
- multi-page HMI flow
- camera / speech / VTK viewer integration

Start the GUI with:

```bash
physical-agent-gui --config configs/arm2_sts3215.example.json
```

If you only want joint-space control, you can also point the GUI at `configs/arm7_sts3215.example.json`.
In that case Cartesian jog will stay disabled because the config has no `urdf_path`.

For `arm2`, the example GUI config uses `cartesian_base_link = base_footprint` instead of
`base_link`. The exported URDF contains a fixed `base_footprint -> base_link` rotation, so using
`base_link` directly makes Cartesian `Y/Z` feel swapped in the GUI.

You can also flip GUI jog directions per axis in the config with
`cartesian_jog_x_sign`, `cartesian_jog_y_sign`, and `cartesian_jog_z_sign`.
The `arm2` example sets `cartesian_jog_z_sign = -1` because its `+Z/-Z` feel inverted with the
current exported model.

## Motion Model

The controller uses `Operating_Mode = 0` and treats `Goal_Position` as an absolute motor-side
target. Each joint exposes a joint-space target in degrees and maps it to motor raw units through
its `gear_ratio`, `direction`, and `zero_position_raw`.

For a commanded joint target:

```text
motor_delta_deg = joint_target_deg * gear_ratio * direction
goal_raw = round(zero_position_raw + motor_delta_deg / 360 * 4096)
```

The controller writes that absolute raw target with:

```python
bus.write("Goal_Position", joint_name, int(goal_raw), normalize=False)
```

Current joint angles are read back from `Present_Position` and converted back into joint-space
degrees with the inverse mapping.

## Cartesian Control

Cartesian control is position-only for now. The controller loads the URDF chain, computes forward
kinematics from the current arm joint feedback, and solves inverse kinematics with a damped least
squares Jacobian update. The solved arm joint targets are then passed through the position-servo
joint controller.

## Assumptions

- Default motor IDs are `1..7`.
- All 7 motors are `sts3215`.
- `joint_2` and `joint_3` use a `14:1` ratio.
- The gripper is servo `7`.
- Position mode is configured at startup.
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

For interactive editing with a 3D viewer, use the PyBullet editor:

```bash
python3 scripts/edit_urdf_limits_pybullet.py \
  models/arm2/urdf/arm2.urdf \
  --limits-file models/arm2/config/joint_limits.json
```

The editor loads the URDF in PyBullet, exposes `pose/lower/upper` sliders for each movable joint,
saves radians into `joint_limits.json`, and can write the updated limits back into the URDF.
