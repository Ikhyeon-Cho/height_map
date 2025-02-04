cmake_minimum_required(VERSION 3.0.2)
project(height_mapping)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    height_mapping_core
    height_mapping_msgs
    height_mapping_io
    height_mapping_utils
)
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    pcl_conversions
    height_mapping_core
    height_mapping_msgs
    height_mapping_io
    height_mapping_utils
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

# Height mapping node
add_executable(height_mapping_node 
  src/nodes/height_mapping_node.cpp
  src/core/HeightMapper.cpp)
target_link_libraries(height_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Global mapping node
add_executable(global_mapping_node 
  src/nodes/global_mapping_node.cpp
  src/core/GlobalMapper.cpp
)
target_link_libraries(global_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Sensor processor node
add_executable(sensor_processor_node 
  src/nodes/sensor_processor_node.cpp)
target_link_libraries(sensor_processor_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)