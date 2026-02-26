#!/usr/bin/env bash
set -Eeuo pipefail
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-02-17
################################################################

IS_REALTIME=false
while [[ $# -gt 0 ]]; do
	case $1 in
	--realtime)
		IS_REALTIME=true
		shift
		;;
	*)
		echo "Unknown option: $1"
		echo "Usage: $0 [--realtime]"
		echo "  --realtime : Run with realtime priority"
		exit 1
		;;
	esac
done

if $IS_REALTIME; then
	echo "Real Time Mode"
	PREFIX="taskset -c 14,15,16,17,18,19,20,21,22,23 chrt 80"
else
	echo "Normal Mode"
	PREFIX=""
fi

CAMERAS_CONFIG="{ \
head: {type: berxel, serial_number: P008GYX6115E3B039, exposure: 16000} \
}"
$PREFIX lerobot-teleoperate \
	--robot.type=hex_arm_sim_follower \
	--robot.headless=True \
	--robot.cameras="$CAMERAS_CONFIG" \
	--teleop.type=hex_dummy_leader \
	--display_data=True \
	--fps=200
