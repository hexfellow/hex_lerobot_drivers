#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-18
################################################################

import time, threading
import cv2
import numpy as np
from collections import deque
from numpy.typing import NDArray
from typing import Any

from berxel_py_wrapper import *

from lerobot.cameras import Camera, ColorMode
from lerobot.cameras.utils import get_cv2_rotation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .configuration_berxel import BerxelCameraConfig


class BerxelCamera(Camera):

    def __init__(self, config: BerxelCameraConfig):
        super().__init__(config)
        self.config = config

        self.serial_number = config.serial_number
        self.exposure = config.exposure
        self.use_depth = config.use_depth

        self.fps = config.fps
        self.color_mode = config.color_mode
        self.warmup_s = max(config.warmup_s, 1)
        self.rotation: int | None = get_cv2_rotation(config.rotation)

        print(
            f"fps: {self.fps}, color_mode: {self.color_mode}, warmup_s: {self.warmup_s}, rotation: {self.rotation}"
        )

        # device
        self.__context = None
        self.__device = None
        self.__intri = np.zeros(4)

        # work loop
        self.__work_event = threading.Event()
        self.__rgb_ready_event = threading.Event()
        self.__depth_ready_event = threading.Event()
        self.__work_thread = threading.Thread(target=self.__work_loop)
        self.__rgb_queue = deque(maxlen=10)
        self.__depth_queue = deque(maxlen=10)

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.serial_number}"

    @property
    def is_connected(self) -> bool:
        return self.__work_event.is_set()

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        # open device
        if not self.__open_device(self.serial_number):
            print("open device failed")
            return

        # start stream
        if not self.__start_stream():
            print("start stream failed")
            return

        start_time = time.time()
        while time.time() - start_time < self.warmup_s:
            hawk_rgb_frame = self.__device.readColorFrame(40)
            hawk_depth_frame = self.__device.readDepthFrame(40)
            if hawk_rgb_frame is not None:
                self.__device.releaseFrame(hawk_rgb_frame)
            if hawk_depth_frame is not None:
                self.__device.releaseFrame(hawk_depth_frame)
            time.sleep(0.01)

        self.__work_event.set()
        self.__work_thread.start()

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        self.__work_event.clear()
        self.__work_thread.join()
        self.__rgb_ready_event.clear()
        self.__depth_ready_event.clear()
        self.__stop_stream()
        self.__close_device()

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        raise NotImplementedError(
            "find_cameras is not implemented for BerxelCamera.")

    def read(self,
             color_mode: ColorMode | None = None,
             timeout_ms: int = 0) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.__rgb_ready_event.wait(timeout_ms / 1000)
        self.__rgb_ready_event.clear()
        if self.use_depth:
            self.__depth_ready_event.wait(timeout_ms / 1000)
            self.__depth_ready_event.clear()
            return self.__rgb_queue[-1], self.__depth_queue[-1]
        else:
            return self.__rgb_queue[-1]

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        rgb = None
        if self.__rgb_ready_event.is_set():
            self.__rgb_ready_event.clear()
            rgb = self.__rgb_queue[-1]

        if self.use_depth:
            if self.__depth_ready_event.is_set():
                depth = self.__depth_queue[-1]
            else:
                depth = None
            return rgb, depth
        else:
            return rgb

    def read_latest(self, max_age_ms: int = 1000) -> NDArray[Any]:
        return self.async_read(max_age_ms)

    def read_depth(self, timeout_ms: int = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.__depth_ready_event.is_set():
            self.__depth_ready_event.clear()
            return self.__depth_queue[-1]
        else:
            return None

    def __work_loop(self) -> None:
        while self.__work_event.is_set():
            # read frame
            hawk_rgb_frame = self.__device.readColorFrame(40)
            hawk_depth_frame = self.__device.readDepthFrame(40)

            # collect rgb frame
            if hawk_rgb_frame is not None:
                frame = self.__unpack_frame(hawk_rgb_frame, False)
                self.__rgb_queue.append(frame)
                self.__rgb_ready_event.set()

            # collect depth frame
            if hawk_depth_frame is not None:
                frame = self.__unpack_frame(hawk_depth_frame, True)
                self.__depth_queue.append(frame)
                self.__depth_ready_event.set()

            self.__device.releaseFrame(hawk_rgb_frame)
            self.__device.releaseFrame(hawk_depth_frame)

    def __open_device(self, serial_number: str | None = None) -> bool:
        # init context
        self.__context = BerxelHawkContext()
        if self.__context is None:
            print("init failed")
            return False
        self.__context.initCamera()

        # open device
        device_list = self.__context.getDeviceList()
        if len(device_list) < 1:
            print("can not find device")
            return False
        if serial_number is not None:
            device_idx = -1

            # check serial number
            def same_serial(tar_serial, device_serial):

                def norm_serial(x):
                    if x is None:
                        return None
                    if isinstance(x, (bytes, bytearray)):
                        x = x.decode('utf-8', 'ignore')
                    x = x.replace('\x00', '').strip()
                    return x.upper()

                return norm_serial(tar_serial) == norm_serial(device_serial)

            for idx, device in enumerate(device_list):
                if same_serial(serial_number, device.serialNumber):
                    print(f"find device with serial number: {serial_number}")
                    device_idx = idx
                    break
            if device_idx == -1:
                print(
                    f"can not find device with serial number: {serial_number}")
                print("available device serial numbers:")
                for device in device_list:
                    print(f"{device.serialNumber}")
                return False
            self.__device = self.__context.openDevice(device_list[device_idx])
        else:
            print("No serial number, use first device")
            self.__device = self.__context.openDevice(device_list[0])

        if self.__device is None:
            print("open device failed")
            return False

        return True

    def __start_stream(self):
        if self.serial_number.startswith('P008'):
            self.__device.setSonixAEStatus(False)
            self.__device.setSonixExposureTime(int(self.exposure // 100))
        else:
            self.__device.setColorExposureGain(self.exposure, 100)
        self.__device.setDepthElectricCurrent(700)
        self.__device.setDepthAE(False)
        self.__device.setDepthExposure(43)
        self.__device.setDepthGain(1)
        self.__device.setRegistrationEnable(True)
        self.__device.setFrameSync(True)
        while self.__device.setSystemClock() != 0:
            print("set system clock failed")
            time.sleep(0.1)

        intrinsic_params = self.__device.getDeviceIntriscParams()
        self.__intri[0] = intrinsic_params.colorIntrinsicParams.fx / 2
        self.__intri[1] = intrinsic_params.colorIntrinsicParams.fy / 2
        self.__intri[2] = intrinsic_params.colorIntrinsicParams.cx / 2
        self.__intri[3] = intrinsic_params.colorIntrinsicParams.cy / 2

        color_frame_mode = self.__device.getCurrentFrameMode(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM'])
        depth_frame_mode = self.__device.getCurrentFrameMode(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'])
        color_frame_mode.framerate = self.fps
        depth_frame_mode.framerate = self.fps
        self.__device.setFrameMode(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM'],
            color_frame_mode)
        self.__device.setFrameMode(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'],
            depth_frame_mode)
        ret = self.__device.startStreams(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM']
            | BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM'])
        if ret == 0:
            print("start stream succeed")
            return True
        else:
            print("start stream failed")
            return False

    def __stop_stream(self):
        if self.__device is None:
            return False
        ret = self.__device.stopStream(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM']
            | BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM'])
        if ret == 0:
            return True
        else:
            return False

    def __close_device(self):
        if self.__context is None:
            return
        if self.__device is None:
            return

        ret = self.__context.closeDevice(self.__device)
        if ret == 0:
            print("clsoe device succeed")
        else:
            print("close device Failed")
        self.__context.destroyCamera()

    def __unpack_frame(
        self,
        hawk_frame: BerxelHawkFrame,
        depth: bool = False,
    ):
        # common variables
        width = hawk_frame.getWidth()
        height = hawk_frame.getHeight()

        if depth:
            # depth frame
            frame_buffer = hawk_frame.getDataAsUint16()
            frame = np.ndarray(
                shape=(height, width),
                dtype=np.uint16,
                buffer=frame_buffer,
            )
            pixel_type = hawk_frame.getPixelType()
            if pixel_type == BerxelHawkPixelType.forward_dict[
                    'BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_12I_4D']:
                frame = frame // 16
            elif pixel_type == BerxelHawkPixelType.forward_dict[
                    'BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D']:
                frame = frame // 8
            else:
                raise ValueError(f"pixel_type: {pixel_type} not supported")
        else:
            # rgb frame
            frame_buffer = hawk_frame.getDataAsUint8()
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=frame_buffer,
            )

        return frame
