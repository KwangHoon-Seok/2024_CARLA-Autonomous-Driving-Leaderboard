# cmake_minimum_required(VERSION 3.0.2)
# project(lidar_processing)

# ## Compile as C++14
# add_compile_options(-std=c++14)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   visualization_msgs
#   pcl_ros
#   pcl_conversions
# )

# catkin_package()

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# add_executable(lidar_downsampling_node src/lidar_downsampling_node.cpp)

# target_link_libraries(lidar_downsampling_node
#   ${catkin_LIBRARIES}
# )




# cmake_minimum_required(VERSION 3.0.2)
# project(lidar_processing)

# ## Compile as C++14
# add_compile_options(-std=c++14)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   visualization_msgs
#   pcl_ros
#   pcl_conversions
# )

# find_package(PCL REQUIRED)

# catkin_package()

# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(lidar_downsampling_node src/lidar_downsampling_node.cpp)

# target_link_libraries(lidar_downsampling_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

cmake_minimum_required(VERSION 3.0.2)
project(lidar_processing)

## Compile as C++14
add_compile_options(-std=c++14)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf2
  tf2_ros
  cv_bridge
  message_generation
)
add_message_files(
  FILES
  DetectionInfo.msg
  DetectionInfoArray.msg
  Tracking.msg
  TrackingArray.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Find PCL (Point Cloud Library)
find_package(PCL REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_processing
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions visualization_msgs tf2 tf2_ros cv_bridge message_runtime
  DEPENDS PCL OpenCV
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Link directories (if needed)
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Declare the executable
add_executable(lidar_downsampling_node src/lidar_downsampling_node.cpp)
add_executable(projection src/projection.cpp)

# Link libraries
target_link_libraries(lidar_downsampling_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(projection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

