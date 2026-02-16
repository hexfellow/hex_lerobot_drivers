#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

import os
import copy
import threading
import numpy as np
from collections import deque
from functools import cached_property
from typing import Any

import mujoco
from mujoco import viewer
from hex_robo_utils import (
    HexDynUtil as DynUtil,
    HexCtrlUtilMitJoint as CtrlUtil,
    HexRate,
)

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceNotConnectedError

from .config_archer_y6_sim import HexArcherY6SimFollowerConfig


class HexArcherY6SimFollower(Robot):
    config_class = HexArcherY6SimFollowerConfig
    name = "hex_archer_y6_sim_follower"

    def __init__(self, config: HexArcherY6SimFollowerConfig):
        super().__init__(config)
        self.config = config

        self.__sim_rate = int(config.sim_rate)
        state_rate = config.state_rate
        image_rate = config.image_rate
        mit_kp = np.ascontiguousarray(np.asarray(config.mit_kp))
        mit_kd = np.ascontiguousarray(np.asarray(config.mit_kd))

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.__work_event = threading.Event()
        self.__ready_event = threading.Event()
        self.__work_thread = threading.Thread(target=self.work_loop)
        self.__obs_queue = deque(maxlen=10)
        self.__cmd_queue = deque(maxlen=10)

        # dyn_util
        urdf_path = os.path.join(script_dir, "urdf", "robot.urdf")
        self.__dyn_util = DynUtil(urdf_path)

        # ctrl util
        self.__mit_kp = np.ascontiguousarray(np.asarray(mit_kp))
        self.__mit_kd = np.ascontiguousarray(np.asarray(mit_kd))
        self.__mit_ctrl = CtrlUtil()

        # mujoco
        model_path = os.path.join(script_dir, "mujoco", "scene.xml")
        self.__model = mujoco.MjModel.from_xml_path(model_path)
        self.__data = mujoco.MjData(self.__model)
        self.__model.opt.timestep = 1.0 / self.__sim_rate
        mujoco.mj_resetData(self.__model, self.__data)
        self.__state_idx = {
            "robot_arm": ([0, 1, 2, 3, 4, 5]),
            "robot_gripper": [6],
            "obj": [12, 13, 14, 15, 16, 17, 18],
        }
        self.__ctrl_idx = {
            "robot_arm": [0, 1, 2, 3, 4, 5],
            "robot_gripper": [6],
        }
        self.__limit_idx = {
            "robot_arm":
            np.arange(len(self.__state_idx["robot_arm"])).tolist(),
            "robot_gripper":
            (np.arange(len(self.__state_idx["robot_gripper"])) +
             len(self.__state_idx["robot_arm"])).tolist(),
        }
        self.__limits = self.__model.jnt_range[np.concatenate(
            [self.__state_idx["robot_arm"], self.
             __state_idx["robot_gripper"]]), :].copy().reshape(-1, 1, 2)
        self.__gripper_ratio = self.__limits[
            self.__state_idx["robot_gripper"], 0,
            1] - self.__limits[self.__state_idx["robot_gripper"], 0, 0]
        self.__dofs = {
            "robot_arm":
            len(self.__state_idx["robot_arm"]),
            "robot_gripper":
            len(self.__state_idx["robot_gripper"]),
            "sum":
            len(self.__state_idx["robot_arm"]) +
            len(self.__state_idx["robot_gripper"]),
        }
        keyframe_id = mujoco.mj_name2id(
            self.__model,
            mujoco.mjtObj.mjOBJ_KEY,
            "home",
        )
        self.__data.qpos = self.__model.key_qpos[keyframe_id]
        self.__data.qvel = np.zeros_like(self.__data.qvel)
        self.__data.ctrl = np.zeros_like(self.__data.ctrl)
        self.__states_trig_thresh = int(self.__sim_rate / state_rate)

        self.__rgb_cam, self.__depth_cam = None, None
        if "fake" in self.config.cameras:
            width = self.config.cameras["fake"].width
            height = self.config.cameras["fake"].height
            self.__rgb_cam = mujoco.Renderer(self.__model, height, width)
            self.__depth_cam = mujoco.Renderer(self.__model, height, width)
            self.__depth_cam.enable_depth_rendering()
        self.__images_trig_thresh = int(self.__sim_rate / image_rate)

        self.__viewer = None

        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        motors = {f"joint_{i}.pos": float for i in range(1, 7)}
        motors["gripper.value"] = float
        return motors

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height,
                  self.config.cameras[cam].width, 3)
            for cam in (self.config.cameras or {})
        }

    @cached_property
    def observation_features(self) -> dict:
        features = {**self._motors_ft, **self._cameras_ft}
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        cam_connected = (all(
            cam.is_connected
            for cam in self.cameras.values()) if self.cameras else True)
        return self.__work_event.is_set() and cam_connected

    def connect(self) -> None:
        mujoco.mj_forward(self.__model, self.__data)
        if not self.config.headless:
            self.__viewer = viewer.launch_passive(self.__model, self.__data)
        else:
            self.__viewer = None

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
        if self.__viewer is not None:
            self.__viewer.close()
            self.__viewer = None
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
        obs = copy.deepcopy(self.__obs_queue[-1])
        if "fake" in self.cameras:
            obs["fake"] = self.cameras["fake"].async_read()

        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.__ready_event.wait()
        self.__cmd_queue.append(action)

    def work_loop(self) -> None:
        rate = HexRate(self.__sim_rate)
        states_trig_count = 0
        images_trig_count = 0
        while self.__work_event.is_set():
            states_trig_count += 1
            images_trig_count += 1

            if states_trig_count >= self.__states_trig_thresh:
                states_trig_count = 0
                cur_qpos = copy.deepcopy(self.__data.qpos)
                cur_qvel = copy.deepcopy(self.__data.qvel)
                arm_q = cur_qpos[self.__state_idx["robot_arm"]]
                arm_dq = cur_qvel[self.__state_idx["robot_arm"]]
                _, c_mat, g_vec, _, _ = self.__dyn_util.dynamic_params(
                    arm_q, arm_dq)
                comp_tor = np.zeros(self.__dofs["sum"])
                comp_tor[:self.__dofs["robot_arm"]] = c_mat @ arm_dq + g_vec

                obs = self.__inner_get_observation(cur_qpos)
                self.__obs_queue.append(obs)

                if not self.__ready_event.is_set():
                    self.__cmd_queue.append(obs)
                    self.__ready_event.set()

                self.__inner_send_action(self.__cmd_queue[-1], comp_tor)

            if images_trig_count >= self.__images_trig_thresh:
                images_trig_count = 0
                self.__rgb_cam.update_scene(self.__data, "end_camera")
                rgb_img = self.__rgb_cam.render()
                # self.__depth_cam.update_scene(self.__data, "end_camera")
                # depth_m = self.__depth_cam.render().astype(np.float32)
                # depth_img = np.clip(depth_m * 1000.0, 0,
                #                     65535).astype(np.uint16)
                if "fake" in self.cameras:
                    self.cameras["fake"].append_rgb(rgb_img)
                    # self.cameras["fake"].append_depth(depth_img)

            mujoco.mj_step(self.__model, self.__data)
            if self.__viewer is not None:
                self.__viewer.sync()
            rate.sleep()

    def __inner_get_observation(self, qpos: np.ndarray) -> dict[str, Any]:
        obs_dict = {}

        # Capture motor positions
        joint_pos = qpos[self.__state_idx["robot_arm"]]
        gripper_pos = qpos[self.__state_idx["robot_gripper"]]
        obs_dict["joint_1.pos"] = float(joint_pos[0])
        obs_dict["joint_2.pos"] = float(joint_pos[1])
        obs_dict["joint_3.pos"] = float(joint_pos[2])
        obs_dict["joint_4.pos"] = float(joint_pos[3])
        obs_dict["joint_5.pos"] = float(joint_pos[4])
        obs_dict["joint_6.pos"] = float(joint_pos[5])
        obs_dict["gripper.value"] = float(
            (gripper_pos[0] -
             self.__limits[self.__state_idx["robot_gripper"], 0, 0]) /
            self.__gripper_ratio)

        return obs_dict

    def __inner_send_action(
        self,
        action: dict[str, Any],
        comp_tor: np.ndarray,
    ) -> None:
        state_idx = self.__state_idx["robot_arm"] + self.__state_idx[
            "robot_gripper"]
        ctrl_idx = self.__ctrl_idx["robot_arm"] + self.__ctrl_idx[
            "robot_gripper"]
        limit_idx = self.__limit_idx["robot_arm"] + self.__limit_idx[
            "robot_gripper"]

        cmd_pos = np.zeros(self.__dofs["sum"])
        tar_vel = np.zeros(self.__dofs["sum"])
        cmd_kp = self.__mit_kp.copy()
        cmd_kd = self.__mit_kd.copy()
        for i in range(self.__dofs["robot_arm"]):
            cmd_pos[i] = action[f"joint_{i+1}.pos"]
        cmd_pos[-self.__dofs["robot_gripper"]:] = action[
            "gripper.value"] * self.__gripper_ratio + self.__limits[
                self.__state_idx["robot_gripper"], 0, 0]
        tar_pos = np.clip(
            cmd_pos,
            self.__limits[limit_idx, 0, 0],
            self.__limits[limit_idx, 0, 1],
        )
        tau_cmds = self.__mit_ctrl(
            cmd_kp,
            cmd_kd,
            tar_pos,
            tar_vel,
            self.__data.qpos[state_idx],
            self.__data.qvel[state_idx],
            comp_tor,
        )
        self.__data.ctrl[ctrl_idx] = tau_cmds
