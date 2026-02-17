#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("hex_hello_leader")
@dataclass
class HexHelloLeaderConfig(TeleoperatorConfig):
    """Configuration for Hello leader arm teleoperator."""

    host: str = "127.0.0.1"
    port: int = 8439
    control_hz: int = 500
