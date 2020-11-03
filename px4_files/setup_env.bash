source ~/catkin_ws/devel/setup.bash
declare FIRMWARE_PATH="/home/neelaksh/catkin_ws/Firmware"
declare VELODYNE_PATH="/home/neelaksh/catkin_ws/src/aq_project/sensors/velodyne_simulator/velodyne_description/models"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${VELODYNE_PATH}
source ${FIRMWARE_PATH}/Tools/setup_gazebo.bash ${FIRMWARE_PATH} ${FIRMWARE_PATH}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_PATH}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_PATH}/Tools/sitl_gazebo