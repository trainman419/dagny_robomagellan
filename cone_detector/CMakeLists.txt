# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(cone_detector)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  sensor_msgs
  tf
  visualization_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/ConeDetector.cfg
  )

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs visualization_msgs dynamic_reconfigure tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cone_detector src/cone_detector.cpp)
add_dependencies(cone_detector ${PROJECT_NAME}_gencfg)
target_link_libraries(cone_detector ${catkin_LIBRARIES})
