cmake_minimum_required(VERSION 3.8)
project(krac_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(krac_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

include_directories(include)

add_executable(vtol_fsm src/vtol_fsm.cpp)
ament_target_dependencies(vtol_fsm
  rclcpp
  mavros_msgs
  geometry_msgs
  krac_interfaces
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_srvs
)

add_executable(vtol_fsm_P src/vtol_fsm_P.cpp)
ament_target_dependencies(vtol_fsm_P
  rclcpp
  mavros_msgs
  geometry_msgs
  krac_interfaces
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_srvs
)

add_executable(precision_lander src/precision_lander.cpp)
ament_target_dependencies(precision_lander
  rclcpp
  geometry_msgs
  krac_interfaces
  std_srvs
  tf2
  tf2_geometry_msgs
)

add_executable(vtol_offboard src/vtol_offboard.cpp)
ament_target_dependencies(vtol_offboard
  rclcpp
  mavros_msgs
  geometry_msgs
  std_srvs
)

add_executable(krac_bt_mock_runner src/krac_bt_mock_runner.cpp)
ament_target_dependencies(krac_bt_mock_runner
  rclcpp
  behaviortree_cpp_v3
)


add_executable(krac_bt_runner
  src/krac_bt_runner.cpp
  src/mission_context.cpp
  src/bt_register.cpp
  src/bt_conditions.cpp
  src/bt_actions_basic.cpp
  src/bt_actions_mission.cpp
  src/bt_actions_vision.cpp
  src/bt_actions_recovery.cpp
  src/bt_actions_precision.cpp
  src/bt_viewer.cpp
)
ament_target_dependencies(krac_bt_runner
  rclcpp
  mavros_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  krac_interfaces
  tf2
  tf2_geometry_msgs
  behaviortree_cpp_v3
)
target_include_directories(krac_bt_runner PRIVATE ${SDL2_INCLUDE_DIRS} ${SDL2_TTF_INCLUDE_DIRS})
target_link_libraries(krac_bt_runner ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})

install(TARGETS
  krac_bt_mock_runner
  krac_bt_runner
  precision_lander
  vtol_fsm
  vtol_fsm_P
  vtol_offboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  bt
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/rescue_controller_placeholder.py
  src/mission_loader.py
  src/gripper_test.py
  src/gripper_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src/
  DESTINATION share/${PROJECT_NAME}/mission
  FILES_MATCHING PATTERN "*.plan"
)

ament_package()
