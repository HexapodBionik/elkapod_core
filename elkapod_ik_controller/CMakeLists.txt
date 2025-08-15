cmake_minimum_required(VERSION 3.20)
project(elkapod_ik_controller LANGUAGES CXX)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  control_toolbox
  controller_interface
  generate_parameter_library
  std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(elkapod_ik_controller_parameters
  src/elkapod_ik_controller_parameter.yaml
  ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp
)


add_library(
  elkapod_ik_controller
  SHARED
  src/elkapod_ik_controller.cpp
  src/elkapod_leg_inverse_kinematics.cpp
)

target_compile_features(elkapod_ik_controller PUBLIC cxx_std_20)
target_include_directories(elkapod_ik_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/elkapod_ik_controller>
)
target_link_libraries(elkapod_ik_controller PUBLIC
  elkapod_ik_controller_parameters
  control_toolbox::control_toolbox
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(controller_interface elkapod_ik_plugin.xml)


install(
    DIRECTORY include/
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS elkapod_ik_controller elkapod_ik_controller_parameters
  EXPORT export_elkapod_ik_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_elkapod_ik_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
