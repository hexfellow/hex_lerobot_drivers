#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

import time
import numpy as np
from typing import Any

from hex_device import HexDeviceApi, Arm, Hands

from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_dummy import HexDummyLeaderConfig


class HexDummyLeader(Teleoperator):

    config_class = HexDummyLeaderConfig
    name = "hex_dummy_leader"

    def __init__(self, config: HexDummyLeaderConfig):
        super().__init__(config)
        self.config = config

        # config
        self.__device_url = f"ws://{config.host}:{config.port}"
        self.__control_hz = config.control_hz

        # variables
        # hex_arm variables
        self.__hex_api: HexDeviceApi | None = None
        self.__arm: Arm | None = None
        self.__gripper: Hands | None = None

        # buffer
        self.__arm_state_buffer: dict | None = None
        self.__gripper_state_buffer: dict | None = None

        # state
        self.__connected_flag = False

    @property
    def action_features(self) -> dict[str, type]:
        return {
            f"joint_{i}.pos": float
            for i in range(1, 7)
        } | {
            f"joint_{i}.vel": float
            for i in range(1, 7)
        } | {
            "gripper.value": float
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.__connected_flag

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        self.__connected_flag = True

    def disconnect(self) -> None:
        if not self.__connected_flag:
            return
        self.__connected_flag = False

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        action = {
            "joint_1.pos": 0.0,
            "joint_2.pos": -1.5,
            "joint_3.pos": 3.0,
            "joint_4.pos": 0.07,
            "joint_5.pos": 0.0,
            "joint_6.pos": 0.0,
            "joint_1.vel": 0.0,
            "joint_2.vel": 0.0,
            "joint_3.vel": 0.0,
            "joint_4.vel": 0.0,
            "joint_5.vel": 0.0,
            "joint_6.vel": 0.0,
            "gripper.value": 0.0,
        }

        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
