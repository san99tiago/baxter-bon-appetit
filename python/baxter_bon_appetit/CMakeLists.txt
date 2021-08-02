cmake_minimum_required(VERSION 2.8.3)
project(baxter_bon_appetit)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  baxter_interface
  cv_bridge
  baxter_core_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  xacro
  actionlib
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/JointSpringsBonAppetit.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  baxter_interface
  cv_bridge
  baxter_core_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  xacro
  actionlib
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

# install(
#   DIRECTORY share/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
#   USE_SOURCE_PERMISSIONS
# )