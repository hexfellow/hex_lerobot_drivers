# lerobot_teleoperator_dummy

LeRobot teleoperator plugin for Dummy leader arm.

## Installation

```bash
pip install lerobot_teleoperator_dummy
```

## Usage

```bash
lerobot-teleoperate \
    --robot.type=hex_arm_sim_follower \
    --robot.headless=False \
    --teleop.type=hex_dummy_leader \
    --display_data=True
```
