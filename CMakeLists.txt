cmake_minimum_required(VERSION 3.8)
project(gst_adapt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

# ---------------------------------------------------------------------------
# Library: gst_adapt_core (hardware detection + pipeline factory)
# ---------------------------------------------------------------------------
add_library(gst_adapt_core SHARED
  src/hardware_detector.cpp
  src/pipeline_factory.cpp
)

target_include_directories(gst_adapt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GST_INCLUDE_DIRS}
)

target_link_directories(gst_adapt_core PUBLIC
  ${GST_LIBRARY_DIRS}
)

# Link GStreamer via ament_target_dependencies to avoid mixing keyword/plain
# target_link_libraries signatures (ament uses plain form internally).
target_link_libraries(gst_adapt_core
  ${GST_LIBRARIES}
)

ament_target_dependencies(gst_adapt_core
  rclcpp
)

# ---------------------------------------------------------------------------
# Component library: gst_adapt_node_component
# ---------------------------------------------------------------------------
add_library(gst_adapt_node_component SHARED
  src/gst_adapt_node.cpp
)

target_link_libraries(gst_adapt_node_component
  gst_adapt_core
)

ament_target_dependencies(gst_adapt_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)

rclcpp_components_register_nodes(gst_adapt_node_component
  "gst_adapt_node::ResizeNode"
)

# ---------------------------------------------------------------------------
# Component library: synthetic_4k_pub_component
# ---------------------------------------------------------------------------
add_library(synthetic_4k_pub_component SHARED
  src/synthetic_4k_pub_node.cpp
)

target_include_directories(synthetic_4k_pub_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(synthetic_4k_pub_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)

rclcpp_components_register_nodes(synthetic_4k_pub_component
  "gst_adapt_node::Synthetic4kPubNode"
)

# ---------------------------------------------------------------------------
# Component library: media_streamer_component
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS videoio)

add_library(media_streamer_component SHARED
  src/media_streamer_node.cpp
)

target_include_directories(media_streamer_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(media_streamer_component
  opencv_videoio
)

ament_target_dependencies(media_streamer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)

rclcpp_components_register_nodes(media_streamer_component
  "gst_adapt_node::MediaStreamerNode"
)

# ---------------------------------------------------------------------------
# Standalone executable
# ---------------------------------------------------------------------------
add_executable(gst_adapt_node
  src/main.cpp
)

target_link_libraries(gst_adapt_node
  gst_adapt_node_component
)

ament_target_dependencies(gst_adapt_node
  rclcpp
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS gst_adapt_core gst_adapt_node_component synthetic_4k_pub_component media_streamer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gst_adapt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/synthetic_4k_pub.py
  scripts/latency_tracker.py
  scripts/cpu_monitor.py
  scripts/visualize_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
