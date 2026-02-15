# hex_lerobot_drivers

Brings a series of integration for HEXFELLOW devices and LeRobot.

## Getting Started

### Installation

```bash
pip install hex_lerobot_drivers
```

If you want to use Berxel devices, install the extras:

```bash
pip install hex_lerobot_drivers[berxel]
```

If you want to use Simulated robots, install the extras:

```bash
pip install hex_lerobot_drivers[mujoco]
```

Or you can just install all extras:

```bash
pip install hex_lerobot_drivers[all]
```

### Usage

Assume you want to use the `hello_follower` teleoperator to control the `fireflyy6_follower` robot, you can run the following command:

```bash
lerobot-teleoperate \
    --robot.type=fireflyy6_follower \
    --teleop.type=hello_follower \
    --fps=60
```

## Devices

| Device                   | Type   | Description                       | Extra  |
| ------------------------ | ------ | --------------------------------- | ------ |
| `firefly_y6_follower`    | Robot  | The robot for Firefly Y6          | \      |
| `archer_y6_follower`     | Robot  | The robot for Archer Y6           | \      |
| `bi_archer_y6_follower`  | Robot  | The robot for double Archer Y6    | \      |
| `archer_y6_sim_follower` | Robot  | The simulated robot for Archer Y6 | mujoco |
| `hello_leader`           | Teleop | The teleoperator for Hello        | \      |
| `bi_hello_leader`        | Teleop | The teleoperator for double Hello | \      |
| `berxel_camera`          | Camera | The camera for Berxel devices     | berxel |

## License

Apache License 2.0. See [LICENSE](LICENSE).

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=hexfellow/hex_lerobot_drivers&type=Date)](https://star-history.com/#hexfellow/hex_lerobot_drivers&Date)

## Contributors

<a href="https://github.com/hexfellow/hex_lerobot_drivers/graphs/contributors">
    <img src="https://contrib.rocks/image?repo=hexfellow/hex_lerobot_drivers" />
</a>
