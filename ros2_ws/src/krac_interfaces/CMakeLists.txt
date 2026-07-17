cmake_minimum_required(VERSION 3.8)
project(krac_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # TargetInfo에서 std_msgs/Header를 쓰므로 필요

# 여기에 방금 만든 msg 파일들을 등록합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlightPhase.msg"
  "msg/TargetInfo.msg"
  "msg/TargetError.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
