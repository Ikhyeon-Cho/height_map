cmake_minimum_required(VERSION 3.0.2)
project(height_mapping_io)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  height_mapping_core
  grid_map_ros
  grid_map_msgs
  pcl_ros
  pcl_conversions
  rosbag
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    height_mapping_core
    grid_map_ros
    grid_map_msgs
    pcl_ros
    pcl_conversions
    rosbag
  DEPENDS
    PCL
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME}
  src/HeightMapReader.cpp
  src/HeightMapWriter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

