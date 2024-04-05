cmake_minimum_required(VERSION 3.0.2)
project(merge_packets)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  velodyne_msgs
  rosbag
)

catkin_package(
 CATKIN_DEPENDS roscpp sensor_msgs velodyne_msgs rosbag
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)