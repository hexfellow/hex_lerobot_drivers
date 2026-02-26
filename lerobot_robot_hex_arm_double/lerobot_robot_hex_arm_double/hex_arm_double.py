#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

import numpy as np
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot_robot_hex_arm import HexArmFollower, HexArmFollowerConfig

from .config_hex_arm_double import HexArmDoubleFollowerConfig


class HexArmDoubleFollower(Robot):

    config_class = HexArmDoubleFollowerConfig
    name = "hex_arm_double_follower"

    def __init__(self, config: HexArmDoubleFollowerConfig):
        super().__init__(config)
        self.config = config

        # config
        left_config = HexArmFollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            host=config.left_config.host,
            port=config.left_config.port,
            control_hz=config.left_config.control_hz,
            arm_type=config.left_config.arm_type,
            gripper_type=config.left_config.gripper_type,
            pos_err_limit=config.left_config.pos_err_limit,
            mit_kp=config.left_config.mit_kp,
            mit_kd=config.left_config.mit_kd,
        )
        right_config = HexArmFollowerConfig(
            id=f"{config.id}_right" if config.id else None,
            host=config.right_config.host,
            port=config.right_config.port,
            control_hz=config.right_config.control_hz,
            arm_type=config.right_config.arm_type,
            gripper_type=config.right_config.gripper_type,
            pos_err_limit=config.right_config.pos_err_limit,
            mit_kp=config.right_config.mit_kp,
            mit_kd=config.right_config.mit_kd,
        )

        # device
        self.__left_follower = HexArmFollower(left_config)
        self.__right_follower = HexArmFollower(right_config)

        # cameras
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _observation_motors_ft(self) -> dict[str, type]:
        left_observation_motors_ft = self.__left_follower._observation_motors_ft
        right_observation_motors_ft = self.__right_follower._observation_motors_ft
        return {
            **{
                f"left_{k}": v
                for k, v in left_observation_motors_ft.items()
            },
            **{
                f"right_{k}": v
                for k, v in right_observation_motors_ft.items()
            },
        }

    @property
    def _action_motors_ft(self) -> dict[str, type]:
        left_action_motors_ft = self.__left_follower._action_motors_ft
        right_action_motors_ft = self.__right_follower._action_motors_ft
        return {
            **{
                f"left_{k}": v
                for k, v in left_action_motors_ft.items()
            },
            **{
                f"right_{k}": v
                for k, v in right_action_motors_ft.items()
            },
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
        return self.__left_follower.is_connected and self.__right_follower.is_connected and cam_connected

    def connect(self) -> None:
        self.__left_follower.connect()
        self.__right_follower.connect()
        for cam in self.cameras.values():
            cam.connect()

    def disconnect(self) -> None:
        self.__left_follower.disconnect()
        self.__right_follower.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return self.__left_follower.is_calibrated and self.__right_follower.is_calibrated

    def calibrate(self) -> None:
        self.__left_follower.calibrate()
        self.__right_follower.calibrate()

    def configure(self) -> None:
        self.__left_follower.configure()
        self.__right_follower.configure()

    def get_observation(self) -> dict[str, Any]:
        left_obs = self.__left_follower.get_observation()
        right_obs = self.__right_follower.get_observation()
        obs_dict = {
            **{
                f"left_{k}": v
                for k, v in left_obs.items()
            },
            **{
                f"right_{k}": v
                for k, v in right_obs.items()
            },
        }

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
        left_action, right_action = {}, {}
        for k, v in action.items():
            if k.startswith("left_"):
                left_action[k.replace("left_", "")] = v
            elif k.startswith("right_"):
                right_action[k.replace("right_", "")] = v

        self.__left_follower.send_action(left_action)
        self.__right_follower.send_action(right_action)
