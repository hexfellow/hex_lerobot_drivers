# lerobot_robot_hex_arm

LeRobot robot plugin for Hex Arm.

## Installation

```bash
pip install lerobot_robot_hex_arm
```

## Usage

```bash
lerobot-teleoperate \
    --robot.type=hex_arm_follower \
    --robot.host=$ARM_HOST \
    --robot.port=$ARM_PORT \
    --robot.arm_type=$ARM_TYPE \
    --robot.gripper_type=$ARM_GRIPPER_TYPE \
    --teleop.type=hex_hello_leader \
    --teleop.host=$HELLO_HOST \
    --teleop.port=$HELLO_PORT \
    --display_data=True \
    --fps=250
```
