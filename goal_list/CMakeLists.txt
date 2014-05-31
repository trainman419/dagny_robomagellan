cmake_minimum_required(VERSION 2.8.3)
project(goal_list)

find_package(catkin REQUIRED COMPONENTS
  dagny_driver
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs dagny_driver
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(goal_list src/goal_list.cpp)
target_link_libraries(goal_list ${catkin_LIBRARIES})
