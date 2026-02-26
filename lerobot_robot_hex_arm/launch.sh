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

ARM_HOST="172.18.24.90"
ARM_PORT=9439
ARM_TYPE="firefly_y6"
ARM_GRIPPER_TYPE="gp80"
HELLO_HOST="172.18.24.90"
HELLO_PORT=8439
CAMERAS_CONFIG="{ \
head: {type: berxel, serial_number: P100RYB4C03M2B322, exposure: 10000} \
}"
$PREFIX lerobot-teleoperate \
	--robot.type=hex_arm_follower \
	--robot.host=$ARM_HOST \
	--robot.port=$ARM_PORT \
	--robot.arm_type=$ARM_TYPE \
	--robot.gripper_type=$ARM_GRIPPER_TYPE \
	--robot.cameras="$CAMERAS_CONFIG" \
	--teleop.type=hex_hello_leader \
	--teleop.host=$HELLO_HOST \
	--teleop.port=$HELLO_PORT \
	--display_data=True \
	--fps=200
