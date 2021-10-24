cmake_minimum_required(VERSION 3.5)
project(gst_h264)

if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Gstreamer doesn't provide CMake files
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(h264_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_executable(
  gst_h264_node
  src/gst_h264_node.cpp
)

ament_target_dependencies(
  gst_h264_node
  camera_info_manager
  GSTREAMER
  GST_APP
  h264_msgs
  rclcpp
)

install(
  TARGETS gst_h264_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
