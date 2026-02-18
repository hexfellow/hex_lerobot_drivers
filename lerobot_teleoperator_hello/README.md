# lerobot_teleoperator_hello

LeRobot teleoperator plugin for Hello leader arm.

## Installation

```bash
pip install lerobot_teleoperator_hello
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
