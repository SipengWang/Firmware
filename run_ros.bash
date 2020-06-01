DONT_RUN=1 make px4_sitl_default gazebo

source ~/catkin_ws/mavros/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
. ~/catkin_ws/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/Firmware ~/catkin_ws/Firmware/build/px4_sitl_default

roslaunch px4 mavros_posix_sitl.launch

