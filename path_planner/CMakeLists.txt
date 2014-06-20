cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  )

find_package(orocos_kdl REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PathPlanner.cfg
  )

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs visualization_msgs std_msgs tf dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(path_planner src/path_planner.cpp)
add_dependencies(path_planner ${PROJECT_NAME}_gencfg)
target_link_libraries(path_planner ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

