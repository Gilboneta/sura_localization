cmake_minimum_required(VERSION 3.8)
project(sura_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sura_msgs REQUIRED)

foreach(target imu_ned_to_enu enu_to_ned_odometry pressure_to_pose ned_to_enu_odometry simple_observer)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    sura_msgs
  )
  target_compile_features(${target} PUBLIC cxx_std_17)
endforeach()

target_link_libraries(simple_observer Eigen3::Eigen)

install(TARGETS
  imu_ned_to_enu
  enu_to_ned_odometry
  pressure_to_pose
  ned_to_enu_odometry
  simple_observer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
