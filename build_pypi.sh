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

cd "$SCRIPT_DIR"

declare -a SUCCESS_DRIVERS=()
declare -a BUILD_FAILED_DRIVERS=()
declare -a UPLOAD_FAILED_DRIVERS=()
declare -a CD_FAILED_DRIVERS=()
declare -a CLEAN_FAILED_DRIVERS=()

while IFS= read -r line || [[ -n "$line" ]]; do
	[[ -z "$line" ]] && continue
	driver="${line%%>=*}"

	cd "$driver" || {
		echo "Skip driver '$driver': failed to cd."
		CD_FAILED_DRIVERS+=("$driver")
		continue
	}

	rm -rf dist build *.egg-info || {
		echo "Warning: clean failed for '$driver', continue."
		CLEAN_FAILED_DRIVERS+=("$driver")
	}

	if ! python3 -m build; then
		echo "Error: build failed for '$driver', skip upload."
		BUILD_FAILED_DRIVERS+=("$driver")
		cd "$SCRIPT_DIR"
		continue
	fi

	if ! twine upload --repository hex --config-file ~/.pypirc dist/* --verbose; then
		echo "Error: upload failed for '$driver', continue."
		UPLOAD_FAILED_DRIVERS+=("$driver")
	else
		SUCCESS_DRIVERS+=("$driver")
	fi

	cd "$SCRIPT_DIR"
done <all_drivers.txt

echo
echo "================ SUMMARY ================"
echo "Successful drivers: ${#SUCCESS_DRIVERS[@]}"
for d in "${SUCCESS_DRIVERS[@]}"; do
	echo "  OK   : $d"
done

echo "Build failed drivers: ${#BUILD_FAILED_DRIVERS[@]}"
for d in "${BUILD_FAILED_DRIVERS[@]}"; do
	echo "  BUILD_FAIL : $d"
done

echo "Upload failed drivers: ${#UPLOAD_FAILED_DRIVERS[@]}"
for d in "${UPLOAD_FAILED_DRIVERS[@]}"; do
	echo "  UPLOAD_FAIL: $d"
done

echo "cd failed drivers: ${#CD_FAILED_DRIVERS[@]}"
for d in "${CD_FAILED_DRIVERS[@]}"; do
	echo "  CD_FAIL    : $d"
done

echo "Clean warned drivers: ${#CLEAN_FAILED_DRIVERS[@]}"
for d in "${CLEAN_FAILED_DRIVERS[@]}"; do
	echo "  CLEAN_WARN : $d"
done
echo "========================================="

cd "$CUR_DIR"
