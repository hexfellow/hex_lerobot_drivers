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
from functools import cached_property
from typing import Any

from hex_device import CommandType, HexDeviceApi, Arm, Hands
from hex_robo_utils import (
    HexDynUtil as DynUtil,
    HexRate,
)

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_hex_arm import HexArmFollowerConfig

ARM_TYPE_TO_INDEX = {
    "archer_y6": 25,
    "firefly_y6": 27,
}


class HexArmFollower(Robot):
    config_class = HexArmFollowerConfig
    name = "hex_arm_follower"

    def __init__(self, config: HexArmFollowerConfig):
        super().__init__(config)
        self.config = config

        # config
        self.__device_url = f"ws://{config.host}:{config.port}"
        self.__control_hz = config.control_hz
        self.__mit_kp = np.ascontiguousarray(np.asarray(config.mit_kp))
        self.__mit_kd = np.ascontiguousarray(np.asarray(config.mit_kd))

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

        # work loop
        self.__work_event = threading.Event()
        self.__ready_event = threading.Event()
        self.__work_thread = threading.Thread(target=self.__work_loop)
        self.__obs_queue = deque(maxlen=10)
        self.__cmd_queue = deque(maxlen=10)

        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _observation_motors_ft(self) -> dict[str, type]:
        return {
            f"joint_{i}.pos": float
            for i in range(1, 7)
        } | {
            f"joint_{i}.vel": float
            for i in range(1, 7)
        } | {
            "gripper.pos": float
        } | {
            "gripper.vel": float
        }

    @property
    def _action_motors_ft(self) -> dict[str, type]:
        return {
            f"joint_{i}.pos": float
            for i in range(1, 7)
        } | {
            f"joint_{i}.vel": float
            for i in range(1, 7)
        } | {
            "gripper.value": float
        }

    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height,
                  self.config.cameras[cam].width, 3)
            for cam in (self.config.cameras or {})
        }

    @cached_property
    def observation_features(self) -> dict:
        features = {**self._observation_motors_ft, **self._cameras_ft}
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._action_motors_ft

    @property
    def is_connected(self) -> bool:
        cam_connected = (all(
            cam.is_connected
            for cam in self.cameras.values()) if self.cameras else True)
        return self.__work_event.is_set() and cam_connected

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        # open device
        self.__hex_api = HexDeviceApi(
            ws_url=self.__device_url,
            control_hz=self.__control_hz,
        )

        # open arm
        while self.__hex_api.find_device_by_robot_type(
                ARM_TYPE_TO_INDEX[self.config.arm_type]) is None:
            print(f"\033[33m{self.config.arm_type} Arm not found\033[0m")
            time.sleep(1)
        self.__arm = self.__hex_api.find_device_by_robot_type(
            ARM_TYPE_TO_INDEX[self.config.arm_type])
        self.__arm.start()
        print(f"\033[32m{self.config.arm_type} Arm connected\033[0m")

        # try to open gripper
        while self.__hex_api.find_optional_device_by_id(1) is None:
            print(
                f"\033[33m{self.config.arm_type} {self.config.gripper_type} not found\033[0m"
            )
            time.sleep(1)
        self.__gripper = self.__hex_api.find_optional_device_by_id(1)
        print(
            f"\033[32m{self.config.arm_type} {self.config.gripper_type} connected\033[0m"
        )

        # get parameters
        arm_dofs, gripper_dofs = len(self.__arm), len(self.__gripper)
        arm_limits = np.array(self.__arm.get_joint_limits()).reshape(-1, 3, 2)
        gripper_limits = np.array(self.__gripper.get_joint_limits()).reshape(
            -1, 3, 2)
        self.__dofs = {
            "robot_arm": arm_dofs,
            "robot_gripper": gripper_dofs,
            "sum": arm_dofs + gripper_dofs,
        }
        self.__motor_idx = {
            "robot_arm": np.arange(arm_dofs).tolist(),
            "robot_gripper": (np.arange(gripper_dofs) + arm_dofs).tolist()
        }
        self.__limits = np.concatenate([arm_limits, gripper_limits], axis=0)
        self.__gripper_ratio = self.__limits[
            self.__motor_idx["robot_gripper"], 0,
            1] - self.__limits[self.__motor_idx["robot_gripper"], 0, 0]
        self.__mit_kp = self.__mit_kp[:self.__dofs["sum"]]
        self.__mit_kd = self.__mit_kd[:self.__dofs["sum"]]

        for cam in self.cameras.values():
            cam.connect()

        self.__work_event.set()
        self.__work_thread.start()

    def disconnect(self) -> None:
        if not self.__work_event.is_set():
            return
        self.__work_event.clear()
        self.__work_thread.join()
        self.__ready_event.clear()
        self.__arm.stop()
        self.__hex_api.close()
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.__ready_event.wait()
        obs_dict = copy.deepcopy(self.__obs_queue[-1])

        for cam_key, cam in self.cameras.items():
            sen_data = cam.async_read()
            if isinstance(sen_data, tuple):
                rgb, depth = sen_data
                if rgb is not None:
                    obs_dict[cam_key] = rgb
                if depth is not None:
                    obs_dict[f"{cam_key}_depth"] = depth
            else:
                if sen_data is not None:
                    obs_dict[cam_key] = sen_data

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.__ready_event.wait()
        self.__cmd_queue.append(action)

    def __work_loop(self) -> None:
        rate = HexRate(self.__control_hz)
        while self.__work_event.is_set():
            rate.sleep()

            self.__read_motor_status()

            obs = self.__inner_get_observation()
            self.__obs_queue.append(obs)

            if not self.__ready_event.is_set():
                init_cmd = {
                    f"joint_{i+1}.pos": float(obs[f"joint_{i+1}.pos"])
                    for i in range(self.__dofs["robot_arm"])
                } | {
                    f"joint_{i+1}.vel": float(obs[f"joint_{i+1}.vel"])
                    for i in range(self.__dofs["robot_arm"])
                } | {
                    "gripper.value":
                    float((obs["gripper.pos"] -
                           self.__limits[self.__motor_idx["robot_gripper"], 0,
                                         0]) / self.__gripper_ratio)
                }
                self.__cmd_queue.append(init_cmd)
                self.__ready_event.set()

            cmd = self.__cmd_queue[-1]
            self.__inner_send_action(cmd)

    def __read_motor_status(self) -> tuple[np.ndarray, np.ndarray]:
        arm_ready, gripper_ready = False, False
        while not arm_ready or not gripper_ready:
            self.__arm_state_buffer = self.__arm.get_simple_motor_status()
            self.__gripper_state_buffer = self.__gripper.get_simple_motor_status(
            )
            arm_ready = self.__arm_state_buffer is not None
            gripper_ready = self.__gripper_state_buffer is not None
            if not arm_ready or not gripper_ready:
                time.sleep(0.1)

    def __inner_get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        arm_pos = self.__arm_state_buffer['pos']
        arm_vel = self.__arm_state_buffer['vel']
        gripper_pos = self.__gripper_state_buffer['pos']
        gripper_vel = self.__gripper_state_buffer['vel']

        obs_dict = {
            f"joint_{i+1}.pos": float(arm_pos[i])
            for i in range(self.__dofs["robot_arm"])
        } | {
            f"joint_{i+1}.vel": float(arm_vel[i])
            for i in range(self.__dofs["robot_arm"])
        } | {
            "gripper.pos": float(float(gripper_pos[0]))
        } | {
            "gripper.vel": float(float(gripper_vel[0]))
        }

        return obs_dict

    def __inner_send_action(
        self,
        cmd: dict[str, Any],
    ) -> None:
        cmd_pos, cmd_vel, cur_pos, cur_vel = self.__parse_action(
            cmd,
            self.config.pos_err_limit,
        )

        arm_q = cur_pos[:self.__dofs["robot_arm"]]
        arm_dq = cur_vel[:self.__dofs["robot_arm"]]
        _, c_mat, g_vec, _, _ = self.__dyn_util.dynamic_params(arm_q, arm_dq)
        comp_tor = np.zeros(self.__dofs["sum"])
        comp_tor[:self.__dofs["robot_arm"]] = c_mat @ arm_dq + g_vec

        # arm control
        arm_cmd = self.__arm.construct_mit_command(
            cmd_pos[:self.__dofs["robot_arm"]],
            cmd_vel[:self.__dofs["robot_arm"]] * 0.5,
            comp_tor[:self.__dofs["robot_arm"]],
            self.__mit_kp[:self.__dofs["robot_arm"]],
            self.__mit_kd[:self.__dofs["robot_arm"]],
        )
        self.__arm.motor_command(CommandType.MIT, arm_cmd)

        # gripper control
        gripper_cmd = self.__gripper.construct_mit_command(
            cmd_pos[-self.__dofs["robot_gripper"]:],
            cmd_vel[-self.__dofs["robot_gripper"]:] * 0.5,
            comp_tor[-self.__dofs["robot_gripper"]:],
            self.__mit_kp[-self.__dofs["robot_gripper"]:],
            self.__mit_kd[-self.__dofs["robot_gripper"]:],
        )
        self.__gripper.motor_command(CommandType.MIT, gripper_cmd)

    def __parse_action(
        self, cmd: dict[str, Any], err_limit: float | None
    ) -> [np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        cur_pos = np.concatenate(
            [
                self.__arm_state_buffer['pos'],
                self.__gripper_state_buffer['pos'],
            ],
            axis=0,
        )
        cur_vel = np.concatenate(
            [
                self.__arm_state_buffer['vel'],
                self.__gripper_state_buffer['vel'],
            ],
            axis=0,
        )

        cmd_pos = np.zeros(self.__dofs["sum"])
        cmd_vel = np.zeros(self.__dofs["sum"])
        for i in range(self.__dofs["robot_arm"]):
            cmd_pos[i] = cmd[f"joint_{i+1}.pos"]
            cmd_vel[i] = cmd[f"joint_{i+1}.vel"]
        cmd_pos[-self.__dofs["robot_gripper"]:] = cmd[
            "gripper.value"] * self.__gripper_ratio + self.__limits[
                self.__motor_idx["robot_gripper"], 0, 0]
        cmd_vel[-self.__dofs["robot_gripper"]:] = 0.0

        if err_limit is not None:
            err = (cmd_pos - cur_pos)[:self.__dofs["robot_arm"]]
            max_err = np.fabs(err).max()
            if max_err > err_limit:
                cmd_pos[:self.__dofs["robot_arm"]] = cur_pos[:self.__dofs[
                    "robot_arm"]] + err * err_limit / max_err

        cmd_pos[:self.__dofs["robot_arm"]] = HexArmFollower.__arm_pos_limits(
            cmd_pos[:self.__dofs["robot_arm"]],
            self.__limits[self.__motor_idx["robot_arm"], 0, 0],
            self.__limits[self.__motor_idx["robot_arm"], 0, 1],
        )
        cmd_pos[
            -self.
            __dofs["robot_gripper"]:] = HexArmFollower.__gripper_pos_limits(
                cmd_pos[-self.__dofs["robot_gripper"]:],
                self.__limits[self.__motor_idx["robot_gripper"], 0, 0],
                self.__limits[self.__motor_idx["robot_gripper"], 0, 1],
            )

        return cmd_pos, cmd_vel, cur_pos, cur_vel

    @staticmethod
    def __rads_normalize(rads: np.ndarray) -> np.ndarray:
        return (rads + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def __arm_pos_limits(
        arm_pos: np.ndarray,
        lower_bound: np.ndarray,
        upper_bound: np.ndarray,
    ) -> np.ndarray:
        normed_rads = HexArmFollower.__rads_normalize(arm_pos)
        outside = (normed_rads < lower_bound) | (normed_rads > upper_bound)
        if not np.any(outside):
            return normed_rads

        lower_dist = np.fabs(
            HexArmFollower.__rads_normalize(
                (normed_rads - lower_bound)[outside]))
        upper_dist = np.fabs(
            HexArmFollower.__rads_normalize(
                (normed_rads - upper_bound)[outside]))
        choose_lower = lower_dist < upper_dist
        choose_upper = ~choose_lower

        outside_full = np.flatnonzero(outside)
        outside_lower = outside_full[choose_lower]
        outside_upper = outside_full[choose_upper]
        normed_rads[outside_lower] = lower_bound[outside_lower]
        normed_rads[outside_upper] = upper_bound[outside_upper]

        return normed_rads

    @staticmethod
    def __gripper_pos_limits(
        gripper_pos: np.ndarray,
        lower_bound: np.ndarray,
        upper_bound: np.ndarray,
    ) -> np.ndarray:
        return np.clip(gripper_pos, lower_bound, upper_bound)
