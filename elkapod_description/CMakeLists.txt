cmake_minimum_required(VERSION 3.20)
project(elkapod_description LANGUAGES CXX)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_library(LIBSERIAL_LIBRARY
             NAMES libserial.so
             PATH_SUFFIXES lib
             PATHS /usr/lib)

# COMPILE
add_library(
  elkapod_legs_control_comm
  SHARED
  hardware/elkapod_legs_system.cpp
  hardware/elkapod_comm.cpp
  hardware/spidev.cpp
)

target_compile_features(elkapod_legs_control_comm PUBLIC cxx_std_20)
target_include_directories(elkapod_legs_control_comm PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/elkapod_legs_control_comm>
)
target_link_libraries(elkapod_legs_control_comm PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  libserial.so
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface elkapod_legs_control_comm.xml)


install(
    DIRECTORY config urdf textures meshes ros2_control hardware/include/
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS elkapod_legs_control_comm
  EXPORT export_elkapod_legs_control_comm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_elkapod_legs_control_comm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
