#!/usr/bin/env bash
set -e

pkill -TERM -x rviz 2>/dev/null || true
pkill -TERM -x gzclient 2>/dev/null || true
pkill -TERM -x gzserver 2>/dev/null || true
pkill -TERM -x move_base 2>/dev/null || true
pkill -TERM -x roslaunch 2>/dev/null || true
pkill -TERM -f '/opt/ros/noetic/lib/tf/tf_echo' 2>/dev/null || true
pkill -TERM -f '/tmp/.*test.*\.py' 2>/dev/null || true
sleep 4

pkill -KILL -x rviz 2>/dev/null || true
pkill -KILL -x gzclient 2>/dev/null || true
pkill -KILL -x gzserver 2>/dev/null || true
pkill -KILL -x move_base 2>/dev/null || true
pkill -KILL -x roslaunch 2>/dev/null || true
pkill -KILL -f '/opt/ros/noetic/lib/tf/tf_echo' 2>/dev/null || true
pkill -KILL -f '/tmp/.*test.*\.py' 2>/dev/null || true
sleep 2

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
export DISPLAY=:0
export DISABLE_ROS1_EOL_WARNINGS=1
exec roslaunch car3 v3_cym_gazebo.launch "$@"
