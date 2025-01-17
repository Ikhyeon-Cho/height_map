cmake_minimum_required(VERSION 3.0.2)
project(height_mapping_data_collection)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  height_mapping_core
  height_mapping_io
  height_mapping_utils
  grid_map_visualization
  pcl_ros
  pcl_conversions
  sensor_msgs
  std_srvs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    height_mapping_core
    height_mapping_io
    height_mapping_utils
    grid_map_visualization
    pcl_ros
    pcl_conversions
    sensor_msgs
    std_srvs
    nav_msgs
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

# Data collection node
add_executable(data_collection_node 
  src/executables/data_collection_main.cpp
  src/nodes/DataCollectionNode.cpp
  src/KITTIScanReader.cpp
  src/KITTIScanWriter.cpp
  src/KITTIMapWriter.cpp
)
target_link_libraries(data_collection_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Data visualization node
add_executable(data_visualization_node 
  src/executables/data_visualization_main.cpp
  src/nodes/DataVisualizationNode.cpp
)
target_link_libraries(data_visualization_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

