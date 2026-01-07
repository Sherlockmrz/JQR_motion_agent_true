# Copyright (c) 2024 系统技术研发三部-硬件中心

cmake_minimum_required(VERSION 3.5)

project(jqr_ros_msgs)

# Default to C++14.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # We don't use add_compile_options with pedantic in message packages
  # because the Python C extensions don't comply with it.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/BatteryLevel.msg"
  "msg/BoundingBox2D.msg"
)

set(srv_files
  "srv/MoveMode.srv"
  "srv/RobotTilt.srv"
  "srv/RobotTiltState.srv"
  "srv/RobotRise.srv"
  "srv/RobotRiseState.srv"
  "srv/ScreenTilt.srv"
  "srv/ScreenTiltState.srv"
  "srv/MedicineBoxSwitch.srv"
  "srv/MedicineBoxState.srv"
  "srv/LaserPointer.srv"
  "srv/LaserPointerState.srv"
  "srv/RgbLightStrip.srv"
  "srv/RgbLightStripState.srv"
  "srv/FaceRecognition.srv"
  "srv/FaceRegistration.srv"
)

set(action_files

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
