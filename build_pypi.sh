#!/usr/bin/env bash
set -Eeuo pipefail
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-26
################################################################

CUR_DIR="$(pwd)"
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "CUR_DIR: $CUR_DIR"
echo "SCRIPT_DIR: $SCRIPT_DIR"

cd $SCRIPT_DIR

if [[ -v VIRTUAL_ENV ]]; then
    echo "In venv, deactivate it First"
else
    rm -rf dist build *.egg-info
    python3 -m build
    twine upload --repository hex --config-file ~/.pypirc  dist/* --verbose
fi

cd $CUR_DIR