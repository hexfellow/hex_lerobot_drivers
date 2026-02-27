# lerobot_teleoperator_hex_arm_double

LeRobot teleoperator plugin for double Hex Arm leader arm.

## Installation

```bash
pip install lerobot_teleoperator_hex_arm_double
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
    --teleop.type=hex_arm_double_leader \
    --teleop.use_mirror_mode=$MIRROR_MODE \
    --teleop.left_config.host=$ARM_HOST \
    --teleop.left_config.port=$LEFT_ARM_PORT \
    --teleop.left_config.arm_type=$ARM_TYPE \
    --teleop.left_config.gripper_type=$ARM_GRIPPER_TYPE \
    --teleop.right_config.host=$ARM_HOST \
    --teleop.right_config.port=$RIGHT_ARM_PORT \
    --teleop.right_config.arm_type=$ARM_TYPE \
    --teleop.right_config.gripper_type=$ARM_GRIPPER_TYPE \
    --display_data=True \
    --fps=200
```
