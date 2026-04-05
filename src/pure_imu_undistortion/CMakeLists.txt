cmake_minimum_required(VERSION 3.14)
project(pure_imu_undistortion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/imu_undistorter.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)


rclcpp_components_register_nodes(${PROJECT_NAME} "pure_imu_undistortion::ImuUndistorter")

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

add_executable(imu_undistorter_node
  src/imu_undistorter_node.cpp
)
target_link_libraries(imu_undistorter_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  imu_undistorter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY param launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs geometry_msgs diagnostic_msgs
  tf2 tf2_ros tf2_geometry_msgs
  Eigen3
)

ament_package()
