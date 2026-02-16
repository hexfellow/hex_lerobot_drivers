#!/usr/bin/env bash
set -Eeuo pipefail
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-16
################################################################

CUR_DIR="$(pwd)"
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "CUR_DIR: $CUR_DIR"
echo "SCRIPT_DIR: $SCRIPT_DIR"

cd $SCRIPT_DIR

while IFS= read -r line || [[ -n "$line" ]]; do
	[[ -z "$line" ]] && continue
	driver="${line%%>=*}"
	cd $driver
	rm -rf dist build *.egg-info
	python3 -m build
	twine upload --repository hex --config-file ~/.pypirc dist/* --verbose
	cd $SCRIPT_DIR
done <all_drivers.txt

cd $CUR_DIR
