# lerobot_robot_hex_arm_double

LeRobot robot plugin for double Hex Arm.

## Installation

```bash
pip install lerobot_robot_hex_arm_double
```

## Usage

```bash
lerobot-teleoperate \
    --robot.type=hex_arm_double_follower \
    --robot.left_config.host=$ARM_HOST \
    --robot.left_config.port=$LEFT_ARM_PORT \
    --robot.left_config.arm_type=$ARM_TYPE \
    --robot.left_config.gripper_type=$ARM_GRIPPER_TYPE \
    --robot.right_config.host=$ARM_HOST \
    --robot.right_config.port=$RIGHT_ARM_PORT \
    --robot.right_config.arm_type=$ARM_TYPE \
    --robot.right_config.gripper_type=$ARM_GRIPPER_TYPE \
    --teleop.type=hex_hello_double_leader \
    --teleop.use_mirror_mode=$MIRROR_MODE \
    --teleop.left_config.host=$HELLO_HOST \
    --teleop.left_config.port=$LEFT_HELLO_PORT \
    --teleop.right_config.host=$HELLO_HOST \
    --teleop.right_config.port=$RIGHT_HELLO_PORT \
    --display_data=True \
    --fps=200
```
