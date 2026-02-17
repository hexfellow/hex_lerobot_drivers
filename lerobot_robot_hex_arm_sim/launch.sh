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

HELLO_HOST="172.18.24.90"
HELLO_PORT=8439
$PREFIX lerobot-teleoperate \
	--robot.type=hex_arm_sim_follower \
	--robot.headless=False \
	--teleop.type=hex_hello_leader \
	--teleop.host=$HELLO_HOST \
	--teleop.port=$HELLO_PORT \
	--display_data=True
