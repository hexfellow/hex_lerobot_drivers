# hex_lerobot_drivers

Brings a series of integration for HEXFELLOW devices and LeRobot.

## Getting Started

### Installation

To install all drivers, run the following command:

```bash
pip install -r all_drivers.txt
```

Or you can install each driver individually:

```bash
pip install lerobot_robot_archer_y6_sim
pip install lerobot_teleoperator_hello
pip install lerobot_camera_berxel
...
```

### Usage

Assume you want to use the `hello` teleoperator to control the simulated `hex_arm` robot, you can run the following command:

```bash
lerobot-teleoperate \
    --robot.type=hex_arm_sim_follower \
    --robot.headless=True \
    --teleop.type=hex_hello_leader \
    --teleop.host="172.18.24.90" \
    --display_data=True
```

## Devices

| Device             | Status |  Type  | Description                         |              pip package              |
| ------------------ | :----: | :----: | ----------------------------------- | :-----------------------------------: |
| **hex_arm**        |   🟢    | Robot  | The robot for Hex Arm               |        `lerobot_robot_hex_arm`        |
| **hex_arm_sim**    |   🟢    | Robot  | The simulated robot for Hex Arm     |      `lerobot_robot_hex_arm_sim`      |
| **hex_arm_double** |   🟢    | Robot  | The robot for double Hex Arm        |    `lerobot_robot_hex_arm_double`     |
| **dummy**          |   🟢    | Teleop | The simulated teleoperator          |     `lerobot_teleoperator_dummy`      |
| **hello**          |   🟢    | Teleop | The teleoperator for Hello          |     `lerobot_teleoperator_hello`      |
| **hello_double**   |   🟢    | Teleop | The teleoperator for double Hello   |  `lerobot_teleoperator_hello_double`  |
| **hex_arm**        |   🟢    | Teleop | The teleoperator for Hex Arm        |    `lerobot_teleoperator_hex_arm`     |
| **hex_arm_double** |   🟢    | Teleop | The teleoperator for double Hex Arm | `lerobot_teleoperator_hex_arm_double` |
| **dummy**          |   🟢    | Camera | The simulated camera                |        `lerobot_camera_dummy`         |
| **berxel**         |   🟢    | Camera | The camera for Berxel devices       |        `lerobot_camera_berxel`        |


## License

Apache License 2.0. See [LICENSE](LICENSE).

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=hexfellow/hex_lerobot_drivers&type=Date)](https://star-history.com/#hexfellow/hex_lerobot_drivers&Date)

## Contributors

<a href="https://github.com/hexfellow/hex_lerobot_drivers/graphs/contributors">
    <img src="https://contrib.rocks/image?repo=hexfellow/hex_lerobot_drivers" />
</a>
