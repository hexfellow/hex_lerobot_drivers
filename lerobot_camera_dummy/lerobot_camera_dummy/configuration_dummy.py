#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-16
################################################################

from dataclasses import dataclass

from lerobot.cameras import CameraConfig, ColorMode, Cv2Rotation


@CameraConfig.register_subclass("hex_dummy_camera")
@dataclass
class HexDummyCameraConfig(CameraConfig):

    width: int = 224
    height: int = 224
    fps: int = 30

    color_mode: ColorMode = ColorMode.BGR
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: int = 1

    def __post_init__(self) -> None:
        if self.color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"`color_mode` is expected to be {ColorMode.RGB.value} or {ColorMode.BGR.value}, but {self.color_mode} is provided."
            )

        if self.rotation not in (
                Cv2Rotation.NO_ROTATION,
                Cv2Rotation.ROTATE_90,
                Cv2Rotation.ROTATE_180,
                Cv2Rotation.ROTATE_270,
        ):
            raise ValueError(
                f"`rotation` is expected to be in {(Cv2Rotation.NO_ROTATION, Cv2Rotation.ROTATE_90, Cv2Rotation.ROTATE_180, Cv2Rotation.ROTATE_270)}, but {self.rotation} is provided."
            )
