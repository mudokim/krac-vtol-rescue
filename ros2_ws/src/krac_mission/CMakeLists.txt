cmake_minimum_required(VERSION 3.8)
project(krac_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 로드
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(krac_interfaces REQUIRED) # [필수] 인터페이스 패키지 로드



# 3-2. 리소스 파일 설치 (launch, config, 폴더 통째로 복사)
install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

# 4. 패키지 종료
ament_package()
