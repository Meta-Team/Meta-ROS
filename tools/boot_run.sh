#!/bin/bash
source ~/.bashrc
file_dir=$(realpath $( dirname $0 ) )
meta_ros_dir=$file_dir/..
echo $meta_ros_dir 
cd $meta_ros_dir
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch meta_bringup hero.launch.py
