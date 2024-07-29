cmake_minimum_required(VERSION 3.8)
project(aidara_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs) 
find_package(std_msgs) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrateCamera.srv"
  "srv/GeometricGrasp.srv"
  "srv/TargetPose.srv"
  "srv/Tf2GetTransform.srv"
  "srv/Tf2TransformPoint.srv"
  "srv/Tf2TransformPose.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
