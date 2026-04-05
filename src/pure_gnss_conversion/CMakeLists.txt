cmake_minimum_required(VERSION 3.8)
project(pure_gnss_conversion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pure_gnss_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
  src/gnss_conversion_main.cpp
  src/gnss_conversion.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pure_gnss_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pure_gnss_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  eigen3_cmake_module
  Eigen3
)

ament_package()
