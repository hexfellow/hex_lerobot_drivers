#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig
from lerobot_teleoperator_hello import HexHelloLeaderConfigBase


@TeleoperatorConfig.register_subclass("hex_hello_double_leader")
@dataclass
class HexHelloDoubleLeaderConfig(TeleoperatorConfig):
    """Configuration for double Hello leader arm teleoperator."""

    left_config: HexHelloLeaderConfigBase
    right_config: HexHelloLeaderConfigBase

    use_mirror_mode: bool = False
