#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig
from lerobot_teleoperator_hex_arm import HexArmLeaderConfigBase


@TeleoperatorConfig.register_subclass("hex_arm_double_leader")
@dataclass
class HexArmDoubleLeaderConfig(TeleoperatorConfig):
    """Configuration for double Hex Arm leader teleoperator."""

    left_config: HexArmLeaderConfigBase
    right_config: HexArmLeaderConfigBase

    use_mirror_mode: bool = False
