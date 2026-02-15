#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-15
################################################################

__all__ = []

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_BERXEL = find_spec("berxel_py_wrapper") is not None

# Optional: berxel
if _HAS_BERXEL:
    from .berxel import BerxelCamera, BerxelCameraConfig
    __all__.extend([
        "BerxelCamera",
        "BerxelCameraConfig",
    ])
