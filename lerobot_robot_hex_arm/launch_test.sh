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

ARM_TYPE="archer_y6"
ARM_GRIPPER_TYPE="gp80"
ARM_HOST="172.18.5.116"
ARM_PORT=9439
FREE_MODE=true
CAMERAS_CONFIG="{ \
head: {type: berxel, serial_number: HK100RB5425M2B024, exposure: 16000} \
}"
$PREFIX lerobot-teleoperate \
	--robot.type=hex_arm_follower \
	--robot.host=$ARM_HOST \
	--robot.port=$ARM_PORT \
	--robot.arm_type=$ARM_TYPE \
	--robot.gripper_type=$ARM_GRIPPER_TYPE \
	--robot.free_mode=$FREE_MODE \
	--robot.cameras="$CAMERAS_CONFIG" \
	--teleop.type=hex_arm_leader \
	--teleop.host=$ARM_HOST \
	--teleop.port=$ARM_PORT \
	--teleop.arm_type=$ARM_TYPE \
	--teleop.gripper_type=$ARM_GRIPPER_TYPE \
	--teleop.read_mode=$FREE_MODE \
	--display_data=True
