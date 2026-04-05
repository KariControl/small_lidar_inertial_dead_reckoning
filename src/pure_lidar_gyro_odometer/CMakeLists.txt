cmake_minimum_required(VERSION 3.14)
project(pure_lidar_gyro_odometer)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(small_gicp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
  src/gyro_odometer_node.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  ndt_omp
  small_gicp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)
if(TARGET small_gicp::small_gicp)
  target_link_libraries(${PROJECT_NAME} small_gicp::small_gicp)
endif()

rclcpp_components_register_nodes(${PROJECT_NAME} "pure_gyro_odometer::GyroOdometerNode")

add_executable(${PROJECT_NAME}_node
  src/gyro_odometer_node_main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

ament_package()
