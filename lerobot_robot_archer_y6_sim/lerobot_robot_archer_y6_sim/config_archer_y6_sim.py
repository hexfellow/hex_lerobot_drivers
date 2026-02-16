#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from dataclasses import dataclass, field

from lerobot.robots import RobotConfig
from lerobot.cameras import Cv2Rotation
from lerobot_camera_fake import FakeCameraConfig


@RobotConfig.register_subclass("hex_archer_y6_sim_follower")
@dataclass
class HexArcherY6SimFollowerConfig(RobotConfig):
    sim_rate: int = 500
    state_rate: int = 500
    image_rate: int = 30
    headless: bool = False
    mit_kp: list[float] = field(
        default_factory=lambda:
        [400.0, 400.0, 500.0, 200.0, 100.0, 100.0, 10.0])
    mit_kd: list[float] = field(
        default_factory=lambda: [5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 0.5])
    cameras: dict[str, FakeCameraConfig] = field(
        default_factory=lambda:
        {"fake": FakeCameraConfig(
            width=224,
            height=224,
            fps=30,
            rotation=Cv2Rotation.ROTATE_270,
        )})
