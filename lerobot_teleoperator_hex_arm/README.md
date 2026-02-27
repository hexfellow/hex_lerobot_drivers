# lerobot_teleoperator_hex_arm

LeRobot teleoperator plugin for Hex Arm leader arm.

## Installation

```bash
pip install lerobot_teleoperator_hex_arm
```

## Usage

```bash
lerobot-teleoperate \
    --robot.type=hex_arm_sim_follower \
    --robot.headless=False \
    --teleop.type=hex_arm_leader \
    --teleop.host=$ARM_HOST \
    --teleop.port=$ARM_PORT \
    --teleop.arm_type=$ARM_TYPE \
    --teleop.gripper_type=$ARM_GRIPPER_TYPE \
    --display_data=True
```
