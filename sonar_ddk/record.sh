#!/bin/sh

if [ $# -eq 0 ]; then
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Recording bag file for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_NAME="dragonfly$MAV_ID"
echo "MAV Name $MAV_NAME"

bag_folder="/sdcard/bags"

if [ ! -d "$bag_folder" ]; then
  echo "*** WARNING *** SD card not present, recording locally"
  cd ~/
else
  echo 'Bag files stored at' $bag_folder
  cd $bag_folder

  #Print out %Used SD card
  USED_PERCENT=$(df --output=pcent $bag_folder | awk '/[0-9]%/{print $(NF)}' | awk '{print substr($1, 1, length($1)-1)}')
  echo 'SD card' ${USED_PERCENT} '% Full'
fi

TOPICS="
/tf
/tf_static"

UKF_TOPICS="
/$MAV_NAME/quadrotor_ukf/control_odom
/$MAV_NAME/quadrotor_ukf/control_odom_throttled
/$MAV_NAME/quadrotor_ukf/imu_bias"

SNAV_TOPICS="
/$MAV_NAME/imu
/$MAV_NAME/imu_raw_array
/$MAV_NAME/vio/internal_states
/$MAV_NAME/vio/map_points
/$MAV_NAME/vio/odometry
/$MAV_NAME/vio/pose
/$MAV_NAME/attitude_estimate"

TOF_TOPICS="
/$MAV_NAME/tof/camera_info
/$MAV_NAME/tof/voxl_depth_image_raw
/$MAV_NAME/tof/voxl_point_cloud"

SONAR_TOPICS="
/sonar_topic"

ALL_TOPICS=$TOPICS$UKF_TOPICS$SNAV_TOPICS$TOF_TOPICS$SONAR_TOPICS

BAG_STAMP=$(date +%F-%H-%M-%S-%Z)
CURR_TIMEZONE=$(date +%Z)

BAG_NAME=$BAG_STAMP-V${MAV_ID}.bag
BAG_PREFIX=V${MAV_ID}-${CURR_TIMEZONE}

eval rosbag record -b512 $ALL_TOPICS -o $BAG_PREFIX