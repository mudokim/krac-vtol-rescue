cmake_minimum_required(VERSION 3.8)
project(krac_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(krac_interfaces REQUIRED)

include_directories(include)

# 실행 파일: competition_logger
add_executable(competition_logger src/competition_logger.cpp)
ament_target_dependencies(competition_logger
  rclcpp
  mavros_msgs
  geometry_msgs
  sensor_msgs
  krac_interfaces
)

install(TARGETS competition_logger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
