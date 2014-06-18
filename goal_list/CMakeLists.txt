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

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(goal_list src/goal_list.cpp src/gps.cpp)
target_link_libraries(goal_list ${catkin_LIBRARIES})
add_dependencies(goal_list ${catkin_EXPORTED_TARGETS})

add_executable(goal_test src/test.cpp src/gps.cpp)
target_link_libraries(goal_test ${catkin_LIBRARIES})
add_dependencies(goal_test ${catkin_EXPORTED_TARGETS})
