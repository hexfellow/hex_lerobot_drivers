#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

import os, time, copy, threading
import numpy as np
from collections import deque
from typing import Any

from hex_device import CommandType, HexDeviceApi, Arm, Hands
from hex_robo_utils import (
    HexDynUtil as DynUtil,
    HexFricUtil as FricUtil,
    HexRate,
)

from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_hex_arm import HexArmLeaderConfig

ARM_TYPE_TO_INDEX = {
    "archer_y6": 25,
    "firefly_y6": 27,
}


class HexArmLeader(Teleoperator):

    config_class = HexArmLeaderConfig
    name = "hex_arm_leader"

    def __init__(self, config: HexArmLeaderConfig):
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
        self.__dofs = None
        self.__limits = None
        self.__gripper_ratio = None

        # dyn_util
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_name = f"{self.config.arm_type}_{self.config.gripper_type}"
        urdf_path = os.path.join(script_dir, "urdf", f"{model_name}.urdf")
        self.__dyn_util = DynUtil(urdf_path)

        # firc_util
        self.__fric_util = FricUtil(
            np.ascontiguousarray(np.asarray(self.config.fric_fc)),
            np.ascontiguousarray(np.asarray(self.config.fric_fv)),
            np.ascontiguousarray(np.asarray(self.config.fric_fo)),
            np.ascontiguousarray(np.asarray(self.config.fric_k)),
        )

        # work loop
        self.__work_event = threading.Event()
        self.__ready_event = threading.Event()
        self.__work_thread = threading.Thread(target=self.__work_loop)
        self.__act_queue = deque(maxlen=10)

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
        return self.__work_event.is_set()

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        # open device
        self.__hex_api = HexDeviceApi(
            ws_url=self.__device_url,
            control_hz=self.__control_hz,
        )

        # open arm
        arm_index = ARM_TYPE_TO_INDEX[self.config.arm_type]
        while self.__hex_api.find_device_by_robot_type(arm_index) is None:
            print(f"\033[33m{self.config.arm_type} Arm not found\033[0m")
            time.sleep(1)
        self.__arm = self.__hex_api.find_device_by_robot_type(arm_index)
        if not self.config.read_mode:
            self.__arm.start()
        print(f"\033[32m{self.config.arm_type} connected\033[0m")

        # try to open gripper
        while self.__hex_api.find_optional_device_by_id(1) is None:
            print(f"\033[33m{self.config.gripper_type} not found\033[0m")
            time.sleep(1)
        self.__gripper = self.__hex_api.find_optional_device_by_id(1)
        print(f"\033[32m{self.config.gripper_type} connected\033[0m")

        # get parameters
        self.__dofs = {
            "robot_arm": len(self.__arm),
            "robot_gripper": len(self.__gripper),
            "robot_sum": len(self.__arm) + len(self.__gripper),
        }
        arm_limits = np.array(self.__arm.get_joint_limits()).reshape(-1, 3, 2)
        gripper_limits = np.array(self.__gripper.get_joint_limits()).reshape(
            -1, 3, 2)

        self.__motor_idx = {
            "robot_arm":
            np.arange(self.__dofs["robot_arm"]).tolist(),
            "robot_gripper":
            np.arange(self.__dofs["robot_arm"],
                      self.__dofs["robot_sum"]).tolist()
        }
        self.__limits = np.concatenate([arm_limits, gripper_limits], axis=0)
        self.__gripper_ratio = self.__limits[
            self.__motor_idx["robot_gripper"], 0,
            1] - self.__limits[self.__motor_idx["robot_gripper"], 0, 0]

        self.__work_event.set()
        self.__work_thread.start()

    def disconnect(self) -> None:
        if not self.__work_event.is_set():
            return
        if not self.config.read_mode:
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

        self.__ready_event.wait()
        act_dict = copy.deepcopy(self.__act_queue[-1])

        return act_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def __work_loop(self) -> None:
        rate = HexRate(self.__control_hz * 2)
        while self.__work_event.is_set():
            rate.sleep()

            new_data = self.__read_motor_status()
            if new_data:
                act = self.__inner_get_action()
                self.__act_queue.append(act)

                if not self.__ready_event.is_set():
                    self.__ready_event.set()

    def __read_motor_status(self) -> tuple[bool, bool]:
        arm_state = self.__arm.get_simple_motor_status()
        gripper_state = self.__gripper.get_simple_motor_status()
        new_arm_ready = arm_state is not None
        new_gripper_ready = gripper_state is not None
        if new_arm_ready:
            self.__arm_state_buffer = arm_state
        if new_gripper_ready:
            self.__gripper_state_buffer = gripper_state
        arm_ready = self.__arm_state_buffer is not None
        gripper_ready = self.__gripper_state_buffer is not None

        new_data = new_arm_ready or new_gripper_ready
        ready = arm_ready and gripper_ready
        return new_data and ready

    def __inner_get_action(self) -> dict[str, Any]:
        arm_pos = self.__arm_state_buffer["pos"]
        arm_vel = self.__arm_state_buffer["vel"]
        gripper_pos = self.__gripper_state_buffer["pos"]
        gripper_vel = self.__gripper_state_buffer["vel"]
        cur_vel = np.concatenate([arm_vel, gripper_vel], axis=0)

        if not self.config.read_mode:
            comp_tor = np.zeros(self.__dofs["robot_sum"])
            comp_tor[:self.__dofs["robot_arm"]] = self.__dyn_util.compensation(
                arm_pos[:self.__dofs["robot_arm"]],
                arm_vel[:self.__dofs["robot_arm"]],
            )
            comp_tor += self.__fric_util(cur_vel)
            arm_cmd = self.__arm.construct_mit_command(
                np.zeros(self.__dofs["robot_arm"]),
                np.zeros(self.__dofs["robot_arm"]),
                comp_tor[:self.__dofs["robot_arm"]],
                np.zeros(self.__dofs["robot_arm"]),
                np.zeros(self.__dofs["robot_arm"]),
            )
            self.__arm.motor_command(CommandType.MIT, arm_cmd)
            gripper_cmd = self.__gripper.construct_mit_command(
                np.zeros(self.__dofs["robot_gripper"]),
                np.zeros(self.__dofs["robot_gripper"]),
                comp_tor[-self.__dofs["robot_gripper"]:],
                np.zeros(self.__dofs["robot_gripper"]),
                np.zeros(self.__dofs["robot_gripper"]),
            )
            self.__gripper.motor_command(CommandType.MIT, gripper_cmd)

        action = {
            f"joint_{i+1}.pos": float(arm_pos[i])
            for i in range(len(arm_pos))
        } | {
            f"joint_{i+1}.vel": float(arm_vel[i])
            for i in range(len(arm_vel))
        } | {
            "gripper.value":
            float(
                np.clip(
                    (gripper_pos[0] -
                     self.__limits[self.__motor_idx["robot_gripper"], 0, 0]) /
                    (self.__limits[self.__motor_idx["robot_gripper"], 0, 1] -
                     self.__limits[self.__motor_idx["robot_gripper"], 0, 0]),
                    0.0,
                    1.0,
                ))
        }
        return action
