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

add_executable(urban_nav src/urban_nav.cpp)
target_link_libraries(urban_nav ${catkin_LIBRARIES})

add_executable(topic src/topic.cpp)
target_link_libraries(topic ${catkin_LIBRARIES})