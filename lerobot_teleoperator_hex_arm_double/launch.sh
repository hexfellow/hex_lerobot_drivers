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
FOLLOWER_HOST="172.18.5.116"
LEFT_FOLLOWER_PORT=8439
RIGHT_FOLLOWER_PORT=9439
LEADER_HOST="172.18.5.116"
LEFT_LEADER_PORT=8439
RIGHT_LEADER_PORT=9439
FREE_MODE=true
MIRROR_MODE=false
CAMERAS_CONFIG="{ \
head: {type: berxel, serial_number: HK100RB5425M2B024, exposure: 16000} \
}"
$PREFIX lerobot-teleoperate \
	--robot.type=hex_arm_double_follower \
	--robot.left_config.host=$FOLLOWER_HOST \
	--robot.left_config.port=$LEFT_FOLLOWER_PORT \
	--robot.left_config.arm_type=$ARM_TYPE \
	--robot.left_config.gripper_type=$ARM_GRIPPER_TYPE \
	--robot.left_config.free_mode=$FREE_MODE \
	--robot.right_config.host=$FOLLOWER_HOST \
	--robot.right_config.port=$RIGHT_FOLLOWER_PORT \
	--robot.right_config.arm_type=$ARM_TYPE \
	--robot.right_config.gripper_type=$ARM_GRIPPER_TYPE \
	--robot.right_config.free_mode=$FREE_MODE \
	--robot.cameras="$CAMERAS_CONFIG" \
	--teleop.type=hex_arm_double_leader \
	--teleop.use_mirror_mode=$MIRROR_MODE \
	--teleop.left_config.host=$LEADER_HOST \
	--teleop.left_config.port=$LEFT_LEADER_PORT \
	--teleop.left_config.arm_type=$ARM_TYPE \
	--teleop.left_config.gripper_type=$ARM_GRIPPER_TYPE \
	--teleop.left_config.read_mode=$FREE_MODE \
	--teleop.right_config.host=$LEADER_HOST \
	--teleop.right_config.port=$RIGHT_LEADER_PORT \
	--teleop.right_config.arm_type=$ARM_TYPE \
	--teleop.right_config.gripper_type=$ARM_GRIPPER_TYPE \
	--teleop.right_config.read_mode=$FREE_MODE \
	--display_data=True
