#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from typing import Any

from lerobot.teleoperators import Teleoperator
from lerobot_teleoperator_hex_arm import HexArmLeader, HexArmLeaderConfig

from .config_hex_arm_double import HexArmDoubleLeaderConfig


class HexArmDoubleLeader(Teleoperator):

    config_class = HexArmDoubleLeaderConfig
    name = "hex_arm_double_leader"

    def __init__(self, config: HexArmDoubleLeaderConfig):
        super().__init__(config)
        self.config = config

        # mirror
        self.__use_mirror_mode = config.use_mirror_mode
        self.__mirror_idx = [1, 5, 6]

        # config
        left_config = HexArmLeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            host=config.left_config.host,
            port=config.left_config.port,
            control_hz=config.left_config.control_hz,
            arm_type=config.left_config.arm_type,
            gripper_type=config.left_config.gripper_type,
            fric_fc=config.left_config.fric_fc,
            fric_fv=config.left_config.fric_fv,
            fric_fo=config.left_config.fric_fo,
            fric_k=config.left_config.fric_k,
            read_mode=config.left_config.read_mode,
        )
        right_config = HexArmLeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            host=config.right_config.host,
            port=config.right_config.port,
            control_hz=config.right_config.control_hz,
            arm_type=config.right_config.arm_type,
            gripper_type=config.right_config.gripper_type,
            fric_fc=config.right_config.fric_fc,
            fric_fv=config.right_config.fric_fv,
            fric_fo=config.right_config.fric_fo,
            fric_k=config.right_config.fric_k,
            read_mode=config.right_config.read_mode,
        )

        # device
        self.__left_leader = HexArmLeader(left_config)
        self.__right_leader = HexArmLeader(right_config)

    @property
    def action_features(self) -> dict[str, type]:
        left_action_features = self.__left_leader.action_features
        right_action_features = self.__right_leader.action_features
        return {
            **{
                f"left_{k}": v
                for k, v in left_action_features.items()
            },
            **{
                f"right_{k}": v
                for k, v in right_action_features.items()
            },
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.__left_leader.is_connected and self.__right_leader.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.__left_leader.connect(calibrate)
        self.__right_leader.connect(calibrate)

    def disconnect(self) -> None:
        self.__left_leader.disconnect()
        self.__right_leader.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return self.__left_leader.is_calibrated and self.__right_leader.is_calibrated

    def calibrate(self) -> None:
        self.__left_leader.calibrate()
        self.__right_leader.calibrate()

    def configure(self) -> None:
        self.__left_leader.configure()
        self.__right_leader.configure()

    def get_action(self) -> dict[str, Any]:
        left_action = self.__left_leader.get_action()
        right_action = self.__right_leader.get_action()

        action = {}
        if self.__use_mirror_mode:
            action = {
                **{
                    f"left_{k}": v
                    for k, v in right_action.items()
                },
                **{
                    f"right_{k}": v
                    for k, v in left_action.items()
                },
            }
            for idx in self.__mirror_idx:
                action[f"left_joint_{idx}.pos"] *= -1.0
                action[f"left_joint_{idx}.vel"] *= -1.0
                action[f"right_joint_{idx}.pos"] *= -1.0
                action[f"right_joint_{idx}.vel"] *= -1.0
        else:
            action = {
                **{
                    f"left_{k}": v
                    for k, v in left_action.items()
                },
                **{
                    f"right_{k}": v
                    for k, v in right_action.items()
                },
            }

        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
