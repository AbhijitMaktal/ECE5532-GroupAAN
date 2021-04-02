cmake_minimum_required(VERSION 2.8.3)
project(urban_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
  ugv_course_libs
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_sim_nhabben src/gps_sim_nhabben.cpp)
target_link_libraries(gps_sim_nhabben ${catkin_LIBRARIES})