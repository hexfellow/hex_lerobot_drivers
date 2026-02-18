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
from lerobot_robot_hex_arm import HexArmFollowerConfigBase


@RobotConfig.register_subclass("hex_arm_double_follower")
@dataclass
class HexArmDoubleFollowerConfig(RobotConfig):
    """Configuration for double Hex Arm."""

    left_config: HexArmFollowerConfigBase
    right_config: HexArmFollowerConfigBase

    cameras: dict[str, CameraConfig] = field(default_factory=dict)