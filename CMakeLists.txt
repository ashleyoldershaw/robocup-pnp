cmake_minimum_required(VERSION 2.8.3)
project(robocupathome_pnp)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  pnp_ros
  tf
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${pnp_ros_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(rcathome_pnpas src/main.cpp src/RCHPNPAS.cpp src/Actions.cpp src/ActionsBase.cpp src/Conditions.cpp)

target_link_libraries(rcathome_pnpas ${pnp_ros_LIBRARIES} ${catkin_LIBRARIES})

