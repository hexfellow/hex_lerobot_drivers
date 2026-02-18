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
	PREFIX="taskset -c 14,15,16,17,18,19,20,21,22,23 chrt 80"
else
	PREFIX=""
fi

ARM_HOST="172.18.22.245"
LEFT_ARM_PORT=8439
RIGHT_ARM_PORT=9439
ARM_TYPE="archer_y6"
ARM_GRIPPER_TYPE="gp80"
HELLO_HOST="172.18.10.251"
LEFT_HELLO_PORT=8439
RIGHT_HELLO_PORT=9439
MIRROR_MODE=true
$PREFIX lerobot-teleoperate \
	--robot.type=hex_arm_double_follower \
	--robot.left_config.host=$ARM_HOST \
	--robot.left_config.port=$LEFT_ARM_PORT \
	--robot.left_config.arm_type=$ARM_TYPE \
	--robot.left_config.gripper_type=$ARM_GRIPPER_TYPE \
	--robot.right_config.host=$ARM_HOST \
	--robot.right_config.port=$RIGHT_ARM_PORT \
	--robot.right_config.arm_type=$ARM_TYPE \
	--robot.right_config.gripper_type=$ARM_GRIPPER_TYPE \
	--teleop.type=hex_hello_double_leader \
	--teleop.use_mirror_mode=$MIRROR_MODE \
	--teleop.left_config.host=$HELLO_HOST \
	--teleop.left_config.port=$LEFT_HELLO_PORT \
	--teleop.right_config.host=$HELLO_HOST \
	--teleop.right_config.port=$RIGHT_HELLO_PORT \
	--display_data=True \
	--fps=250
