cmake_minimum_required(VERSION 3.8)
project(sura_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/controllers/madgwick_controller.cpp
  src/controllers/madgwick_filter.cpp
  src/controllers/ekf_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

add_executable(quaternion_to_rpy_node
  src/nodes/quaternion_to_rpy_node.cpp
)
target_compile_features(quaternion_to_rpy_node PUBLIC cxx_std_17)
ament_target_dependencies(quaternion_to_rpy_node
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(
  TARGETS quaternion_to_rpy_node
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  controller_interface
  sura_localization_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES sura_localization_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

ament_package()
