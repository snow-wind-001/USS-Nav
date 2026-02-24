cmake_minimum_required(VERSION 3.8)
project(uss_nav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  "msg/RollingGrid3D.msg"
  "msg/Frontier.msg"
  "msg/FrontierArray.msg"
  "msg/SCGNode.msg"
  "msg/SCGEdge.msg"
  "msg/SCGGraph.msg"
  "msg/RegionLabel.msg"
  "msg/RegionLabelArray.msg"
  "msg/ObjectNode.msg"
  "msg/ObjectGraph.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
