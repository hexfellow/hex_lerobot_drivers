#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from dataclasses import dataclass, field

from lerobot.robots import RobotConfig
from lerobot.cameras import CameraConfig


@dataclass
class HexArmFollowerConfigBase:
    host: str = "127.0.0.1"
    port: int = 8439
    control_hz: int = 500

    arm_type: str = "firefly_y6"
    gripper_type: str = "gp80"

    pos_err_limit: float = 0.1
    mit_kp: list[float] = field(
        default_factory=lambda:
        [300.0, 300.0, 350.0, 150.0, 100.0, 100.0, 10.0])
    mit_kd: list[float] = field(
        default_factory=lambda: [5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 0.5])


@RobotConfig.register_subclass("hex_arm_follower")
@dataclass
class HexArmFollowerConfig(RobotConfig, HexArmFollowerConfigBase):
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
