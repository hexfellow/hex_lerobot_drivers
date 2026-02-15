#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from .robots import HexFireflyY6Follower, HexFireflyY6FollowerConfig
from .robots import HexArcherY6Follower, HexArcherY6FollowerConfig
from .robots import HexBiArcherY6Follower, HexBiArcherY6FollowerConfig

from .teleoperators import HexHelloLeader, HexHelloLeaderConfig
from .teleoperators import HexBiHelloLeader, HexBiHelloLeaderConfig

__all__ = [
    # robots
    "HexArcherY6Follower",
    "HexArcherY6FollowerConfig",
    "HexBiArcherY6Follower",
    "HexBiArcherY6FollowerConfig",
    "HexFireflyY6Follower",
    "HexFireflyY6FollowerConfig",

    # teleoperators
    "HexHelloLeader",
    "HexHelloLeaderConfig",
    "HexBiHelloLeader",
    "HexBiHelloLeaderConfig",
]

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_BERXEL = find_spec("berxel_py_wrapper") is not None
_HAS_MUJOCO = find_spec("mujoco") is not None

# Optional: berxel
if _HAS_BERXEL:
    from .cameras import HexBerxelCamera, HexBerxelCameraConfig
    __all__.extend([
        # cameras
        "HexBerxelCamera",
        "HexBerxelCameraConfig",
    ])

# Optional: mujoco
if _HAS_MUJOCO:
    from .robots import HexArcherY6SimFollower, HexArcherY6SimFollowerConfig
    __all__.extend([
        # robots
        "HexArcherY6SimFollower",
        "HexArcherY6SimFollowerConfig",
    ])
