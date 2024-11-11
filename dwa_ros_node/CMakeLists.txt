cmake_minimum_required(VERSION 3.0.2)
project(dwa_ros_node)

# C++14 사용을 명시
add_compile_options(-std=c++14)

# catkin 패키지 구성
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  carla_msgs
  message_generation
)

# Eigen3 라이브러리 찾기
find_package(Eigen3 REQUIRED)

# 포함할 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
  FILES
  DetectionInfo.msg
  DetectionInfoArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
# 실행 파일 및 링크 라이브러리 설정
add_executable(dwa_node src/dwa_node.cpp)

target_link_libraries(dwa_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 설치 설정
install(TARGETS dwa_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ROS 노드 설정
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf carla_msgs message_generation
)
