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

Assume you want to use the `hello` teleoperator to control the simulated `archer_y6` robot, you can run the following command:

```bash
lerobot-teleoperate \
    --robot.type=hex_archer_y6_sim_follower \
    --robot.headless=True \
    --teleop.type=hex_hello_leader \
    --teleop.host="172.18.24.90" \
    --display_data=True
```

## Devices

| Device            | Status |  Type  | Description                       |           pip package           |
| ----------------- | :----: | :----: | --------------------------------- | :-----------------------------: |
| **firefly_y6**    |   🟡    | Robot  | The robot for Firefly Y6          |   `lerobot_robot_firefly_y6`    |
| **archer_y6**     |   🟡    | Robot  | The robot for Archer Y6           |    `lerobot_robot_archer_y6`    |
| **bi_archer_y6**  |   🟡    | Robot  | The robot for double Archer Y6    |  `lerobot_robot_bi_archer_y6`   |
| **archer_y6_sim** |   🟢    | Robot  | The simulated robot for Archer Y6 |  `lerobot_robot_archer_y6_sim`  |
| **hello**         |   🟢    | Teleop | The teleoperator for Hello        |  `lerobot_teleoperator_hello`   |
| **bi_hello**      |   🟡    | Teleop | The teleoperator for double Hello | `lerobot_teleoperator_bi_hello` |
| **fake**          |   🟢    | Camera | The simulated camera              |      `lerobot_camera_fake`      |
| **berxel**        |   🟡    | Camera | The camera for Berxel devices     |     `lerobot_camera_berxel`     |

## License

Apache License 2.0. See [LICENSE](LICENSE).

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=hexfellow/hex_lerobot_drivers&type=Date)](https://star-history.com/#hexfellow/hex_lerobot_drivers&Date)

## Contributors

<a href="https://github.com/hexfellow/hex_lerobot_drivers/graphs/contributors">
    <img src="https://contrib.rocks/image?repo=hexfellow/hex_lerobot_drivers" />
</a>
