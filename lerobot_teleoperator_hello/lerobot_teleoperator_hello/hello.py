#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

import time
from typing import Any
import numpy as np

from hex_device import HexDeviceApi, Arm, Hands

from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_hello import HexHelloLeaderConfig


class HexHelloLeader(Teleoperator):

    config_class = HexHelloLeaderConfig
    name = "hex_hello_leader"

    def __init__(self, config: HexHelloLeaderConfig):
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

        # open device
        self.__hex_api = HexDeviceApi(
            ws_url=self.__device_url,
            control_hz=self.__control_hz,
        )

        # open arm
        while self.__hex_api.find_device_by_robot_type(26) is None:
            print("\033[33mArm not found\033[0m")
            time.sleep(1)
        self.__arm = self.__hex_api.find_device_by_robot_type(26)
        self.__arm.start()

        # try to open gripper
        self.__gripper = self.__hex_api.find_optional_device_by_id(1)
        if self.__gripper is None:
            print("\033[33mGripper not found\033[0m")
        self.__gripper.set_rgb_stripe_command([0] * 6, [255] * 6, [0] * 6)

        self.__connected_flag = True

    def disconnect(self) -> None:
        if not self.__connected_flag:
            return
        self.__connected_flag = False
        self.__gripper.set_rgb_stripe_command([255] * 6, [0] * 6, [0] * 6)
        time.sleep(0.2)
        self.__arm.stop()
        self.__hex_api.close()

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

        arm_ready, gripper_ready = False, False
        while not arm_ready or not gripper_ready:
            self.__arm_state_buffer = self.__arm.get_simple_motor_status()
            self.__gripper_state_buffer = self.__gripper.get_simple_motor_status(
            )
            arm_ready = self.__arm_state_buffer is not None
            gripper_ready = self.__gripper_state_buffer is not None
            if not arm_ready or not gripper_ready:
                time.sleep(0.1)

        arm_pos = self.__arm_state_buffer['pos']
        gripper_pos = self.__gripper_state_buffer['pos']

        action = {
            f"joint_{i+1}.pos": float(arm_pos[i])
            for i in range(len(arm_pos))
        } | {
            "gripper.value": float(np.clip((gripper_pos[0] + 1.0) * 0.5, 0.0, 1.0))
        }
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
