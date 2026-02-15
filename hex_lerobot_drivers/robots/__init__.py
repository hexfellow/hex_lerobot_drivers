#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

from .firefly_y6_follower import HexFireflyY6Follower, HexFireflyY6FollowerConfig
from .archer_y6_follower import HexArcherY6Follower, HexArcherY6FollowerConfig
from .bi_archer_y6_follower import HexBiArcherY6Follower, HexBiArcherY6FollowerConfig

__all__ = [
    # firefly y6 follower
    "HexFireflyY6Follower",
    "HexFireflyY6FollowerConfig",

    # archer y6 follower
    "HexArcherY6Follower",
    "HexArcherY6FollowerConfig",

    # bi archer y6 follower
    "HexBiArcherY6Follower",
    "HexBiArcherY6FollowerConfig",
]

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_MUJOCO = find_spec("mujoco") is not None

# Optional: mujoco
if _HAS_MUJOCO:
    from .archer_y6_sim_follower import HexArcherY6SimFollower, HexArcherY6SimFollowerConfig
    __all__.extend([
        "HexArcherY6SimFollower",
        "HexArcherY6SimFollowerConfig",
    ])
