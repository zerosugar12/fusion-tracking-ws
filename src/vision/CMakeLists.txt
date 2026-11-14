cmake_minimum_required(VERSION 3.12)
project(vision LANGUAGES CXX)

if(EXISTS "/usr/local/cuda/bin/nvcc")
  set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda" CACHE PATH "CUDA toolkit root (pinned)" FORCE)
  set(CUDAToolkit_ROOT "/usr/local/cuda" CACHE PATH "CUDAToolkit root (pinned)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

# ZED SDK & CUDA Direct Integration
find_package(ZED 3 REQUIRED)
find_package(CUDAToolkit REQUIRED)

include_directories(
  include
  ${ZED_INCLUDE_DIRS}
  ${CUDAToolkit_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Single Unified Tracking Node
add_executable(fusion_tracker_node
  src/leg_detector.cpp
  src/csv_logger.cpp
  src/fusion_tracker_node.cpp
)
ament_target_dependencies(fusion_tracker_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(fusion_tracker_node
  ${CUDA_cudart_LIBRARY}
  ${OpenCV_LIBS}
  ${ZED_LIBRARIES}
)

# Install binary
install(TARGETS
  fusion_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include
)
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/plot_benchmark.py")
  install(PROGRAMS
    scripts/plot_benchmark.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
