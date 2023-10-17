#! /bin/bash

wspath="/home/jetson/uav/uav_ws"

ID="1"
GCS_IP="192.168.1.99"
GCS_PORT=$((15000+$ID))
FCU_PORT="ttyUSB0"
BAUD="921600"
NAMESPACE="m690b${ID}/mavros"

OPTIONS="fcu_url:=/dev/${FCU_PORT}:${BAUD} gcs_url:=udp://@${GCS_IP}:${GCS_PORT} tgt_system:=${ID} ID:=${ID} GCS_IP:=${GCS_IP} GCS_PORT:=${GCS_PORT}"

source /opt/ros/noetic/setup.bash
source $wspath/devel/setup.bash
roslaunch drone_launch mavros_run.launch ${OPTIONS}
