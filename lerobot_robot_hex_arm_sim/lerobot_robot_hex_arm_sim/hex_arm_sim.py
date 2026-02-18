#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

import os, copy, threading
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

from .config_hex_arm_sim import HexArmSimFollowerConfig


class HexArmSimFollower(Robot):
    config_class = HexArmSimFollowerConfig
    name = "hex_arm_sim_follower"

    def __init__(self, config: HexArmSimFollowerConfig):
        super().__init__(config)
        self.config = config

        self.__sim_rate = int(config.control_hz)
        state_rate = config.state_rate
        image_rate = config.image_rate

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.__work_event = threading.Event()
        self.__ready_event = threading.Event()
        self.__work_thread = threading.Thread(target=self.__work_loop)
        self.__obs_queue = deque(maxlen=10)
        self.__cmd_queue = deque(maxlen=10)

        # dyn_util
        urdf_path = os.path.join(script_dir, "urdf", "robot.urdf")
        self.__dyn_util = DynUtil(urdf_path)

        # ctrl util
        self.__mit_kp = np.ascontiguousarray(np.asarray(self.config.mit_kp))
        self.__mit_kd = np.ascontiguousarray(np.asarray(self.config.mit_kd))
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
        if "dummy" in self.config.cameras:
            width = self.config.cameras["dummy"].width
            height = self.config.cameras["dummy"].height
            self.__rgb_cam = mujoco.Renderer(self.__model, height, width)
            self.__depth_cam = mujoco.Renderer(self.__model, height, width)
            self.__depth_cam.enable_depth_rendering()
        self.__images_trig_thresh = int(self.__sim_rate / image_rate)

        self.__viewer = None

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

    @property
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
        obs_dict = copy.deepcopy(self.__obs_queue[-1])
        
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.__ready_event.wait()
        self.__cmd_queue.append(action)

    def __work_loop(self) -> None:
        rate = HexRate(self.__sim_rate)
        states_trig_count = 0
        images_trig_count = 0
        while self.__work_event.is_set():
            states_trig_count += 1
            images_trig_count += 1

            if states_trig_count >= self.__states_trig_thresh:
                states_trig_count = 0

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
                               self.__limits[self.__state_idx["robot_gripper"],
                                             0, 0]) / self.__gripper_ratio)
                    }
                    self.__cmd_queue.append(init_cmd)
                    self.__ready_event.set()

                cmd = self.__cmd_queue[-1]
                self.__inner_send_action(cmd)

            
            if images_trig_count >= self.__images_trig_thresh:
                images_trig_count = 0
                if "dummy" in self.cameras:
                    self.__rgb_cam.update_scene(self.__data, "end_camera")
                    rgb_img = self.__rgb_cam.render()
                    self.cameras["dummy"].append_rgb(rgb_img)
                    # self.__depth_cam.update_scene(self.__data, "end_camera")
                    # depth_m = self.__depth_cam.render().astype(np.float32)
                    # depth_img = np.clip(depth_m * 1000.0, 0,
                    #                     65535).astype(np.uint16)
                    # self.cameras["dummy"].append_depth(depth_img)

            mujoco.mj_step(self.__model, self.__data)
            if self.__viewer is not None:
                self.__viewer.sync()
            rate.sleep()

    def __inner_get_observation(self) -> dict[str, Any]:
        obs_dict = {}

        arm_pos = self.__data.qpos[self.__state_idx["robot_arm"]]
        arm_vel = self.__data.qvel[self.__state_idx["robot_arm"]]
        gripper_pos = self.__data.qpos[self.__state_idx["robot_gripper"]]
        gripper_vel = self.__data.qvel[self.__state_idx["robot_gripper"]]

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

        tau_cmds = self.__mit_ctrl(
            self.__mit_kp,
            self.__mit_kd,
            cmd_pos,
            cmd_vel,
            cur_pos,
            cur_vel,
            comp_tor,
        )
        ctrl_idx = self.__ctrl_idx["robot_arm"] + self.__ctrl_idx[
            "robot_gripper"]
        self.__data.ctrl[ctrl_idx] = tau_cmds

    def __parse_action(
        self, cmd: dict[str, Any], err_limit: float | None
    ) -> [np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        cur_pos = self.__data.qpos[self.__state_idx["robot_arm"] +
                                   self.__state_idx["robot_gripper"]]
        cur_vel = self.__data.qvel[self.__state_idx["robot_arm"] +
                                   self.__state_idx["robot_gripper"]]

        cmd_pos = np.zeros(self.__dofs["sum"])
        cmd_vel = np.zeros(self.__dofs["sum"])
        for i in range(self.__dofs["robot_arm"]):
            cmd_pos[i] = cmd[f"joint_{i+1}.pos"]
            cmd_vel[i] = cmd[f"joint_{i+1}.vel"]
        cmd_pos[-self.__dofs["robot_gripper"]:] = cmd[
            "gripper.value"] * self.__gripper_ratio + self.__limits[
                self.__state_idx["robot_gripper"], 0, 0]
        cmd_vel[-self.__dofs["robot_gripper"]:] = 0.0

        if err_limit is not None:
            err = (cmd_pos - cur_pos)[:self.__dofs["robot_arm"]]
            max_err = np.fabs(err).max()
            if max_err > err_limit:
                cmd_pos[:self.__dofs["robot_arm"]] = cur_pos[:self.__dofs[
                    "robot_arm"]] + err * err_limit / max_err

        cmd_pos[:self.
                __dofs["robot_arm"]] = HexArmSimFollower.__arm_pos_limits(
                    cmd_pos[:self.__dofs["robot_arm"]],
                    self.__limits[self.__state_idx["robot_arm"], 0, 0],
                    self.__limits[self.__state_idx["robot_arm"], 0, 1],
                )
        cmd_pos[
            -self.
            __dofs["robot_gripper"]:] = HexArmSimFollower.__gripper_pos_limits(
                cmd_pos[-self.__dofs["robot_gripper"]:],
                self.__limits[self.__state_idx["robot_gripper"], 0, 0],
                self.__limits[self.__state_idx["robot_gripper"], 0, 1],
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
        normed_rads = HexArmSimFollower.__rads_normalize(arm_pos)
        outside = (normed_rads < lower_bound) | (normed_rads > upper_bound)
        if not np.any(outside):
            return normed_rads

        lower_dist = np.fabs(
            HexArmSimFollower.__rads_normalize(
                (normed_rads - lower_bound)[outside]))
        upper_dist = np.fabs(
            HexArmSimFollower.__rads_normalize(
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
