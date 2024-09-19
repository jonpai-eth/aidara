cmake_minimum_required(VERSION 3.5)
project(compliant_trajectory_controller)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
set(REF_GEN_INCLUDE_DEPENDS
  rclcpp
  Eigen3
)
set(CONTROLLER_INCLUDE_DEPENDS
  aidara_msgs
  controller_interface
  franka_msgs
  franka_semantic_components
  pinocchio
  rclcpp_action
  rcpputils
  realtime_tools
  std_srvs
  trajectory_msgs
  tf2_ros
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE

add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)


add_library(
  compliant_trajectory_controller
  SHARED
  controller/compliant_trajectory_controller.cpp
)

target_compile_features(compliant_trajectory_controller PUBLIC cxx_std_17)
target_include_directories(compliant_trajectory_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/compliant_trajectory_controller>
)
ament_target_dependencies(
  compliant_trajectory_controller PUBLIC
  franka_semantic_components
  franka_msgs
  rclcpp_action
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "Trajectory_Controller")

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface compliant_trajectory_controller_plugin.xml)

# INSTALL

install(TARGETS compliant_trajectory_controller
  EXPORT compliant_trajectory_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY controller/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(compliant_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package() 

