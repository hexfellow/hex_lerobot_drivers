# lerobot_camera_dummy

LeRobot simulated camera plugin for Hex Arm (MuJoCo).

## Installation

```bash
pip install lerobot_camera_dummy
```

## Usage

```bash
lerobot-teleoperate \
    --robot.type=hex_arm_sim_follower \
    --robot.headless=False \
    --teleop.type=hex_hello_leader \
    --teleop.host=$HELLO_HOST \
    --teleop.port=$HELLO_PORT \
    --display_data=True
```
