#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-16
################################################################

import time
from typing import Any

import cv2
import numpy as np
from collections import deque
from numpy.typing import NDArray

from lerobot.cameras import Camera, ColorMode
from lerobot.cameras.utils import get_cv2_rotation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .configuration_dummy import HexDummyCameraConfig


class HexDummyCamera(Camera):

    def __init__(self, config: HexDummyCameraConfig):
        super().__init__(config)
        self.config = config

        self.fps = config.fps
        self.color_mode = config.color_mode
        self.warmup_s = max(config.warmup_s, 1)
        self.rotation: int | None = get_cv2_rotation(config.rotation)

        self.__rgb_queue = deque(maxlen=10)
        self.__depth_queue = deque(maxlen=10)

        self.connected_flag = False

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.config.name}, {self.config.image_type})"

    @property
    def is_connected(self) -> bool:
        return self.connected_flag

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        if not self.__rgb_queue:
            self.__rgb_queue.append(
                (time.perf_counter(),
                 np.zeros((self.config.height, self.config.width, 3),
                          dtype=np.uint8)))
        if not self.__depth_queue:
            self.__depth_queue.append(
                (time.perf_counter(),
                 np.zeros((self.config.height, self.config.width),
                          dtype=np.uint16)))

        start_time = time.time()
        while time.time() - start_time < self.warmup_s:
            time.sleep(0.1)

        self.connected_flag = True

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        self.connected_flag = False

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        raise NotImplementedError(
            "find_cameras is not implemented for HexDummyCamera.")

    def read(self,
             color_mode: ColorMode | None = None,
             timeout_ms: int = 0) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        _, rgb = self.__rgb_queue[-1]
        return rgb

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        _, rgb = self.__rgb_queue[-1]
        return rgb

    def read_latest(self, max_age_ms: int = 1000) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        _, rgb = self.__rgb_queue[-1]
        return rgb

    def read_depth(self, timeout_ms: int = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        _, depth = self.__depth_queue[-1]
        return depth

    def append_rgb(self, rgb: NDArray[Any]) -> None:
        ts = time.perf_counter()
        rotated_rgb = cv2.rotate(rgb, self.rotation)
        self.__rgb_queue.append((ts, rotated_rgb))

    def append_depth(self, depth: NDArray[Any]) -> None:
        ts = time.perf_counter()
        rotated_depth = cv2.rotate(depth, self.rotation)
        self.__depth_queue.append((ts, rotated_depth))
