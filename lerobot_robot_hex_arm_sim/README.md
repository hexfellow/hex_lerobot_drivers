# lerobot_robot_hex_arm_sim

LeRobot simulated robot plugin for Hex Arm (MuJoCo).

## Installation

```bash
pip install lerobot_robot_hex_arm_sim
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
