#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from dataclasses import dataclass, field

from lerobot.teleoperators import TeleoperatorConfig


@dataclass
class HexArmLeaderConfigBase:
    """Base configuration for Hex Arm leader teleoperator."""

    host: str = "127.0.0.1"
    port: int = 8439
    control_hz: int = 500

    # Hex Arm specific parameters
    arm_type: str = "firefly_y6"
    gripper_type: str = "gp80"

    fric_fc: list[float] = field(
        default_factory=lambda: [0.1, 0.1, 0.3, 0.3, 0.0, 0.0, 0.05])
    fric_fv: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    fric_fo: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    fric_k: list[float] = field(
        default_factory=lambda:
        [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 400.0])
    read_mode: bool = False


@TeleoperatorConfig.register_subclass("hex_arm_leader")
@dataclass
class HexArmLeaderConfig(TeleoperatorConfig, HexArmLeaderConfigBase):
    """Configuration for Hex Arm leader teleoperator."""

    pass
