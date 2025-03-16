cmake_minimum_required(VERSION 3.16)
project(elkapod_gait_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable
          -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
          -Werror=missing-braces)
endif()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        controller_interface
        hardware_interface
        generate_parameter_library
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(elkapod_gait_controller_parameters
        src/elkapod_gait_controller_parameters.yaml
)

add_library(elkapod_gait_controller SHARED
        src/elkapod_gait_controller.cpp
)

target_compile_features(elkapod_gait_controller PUBLIC cxx_std_20)
target_include_directories(elkapod_gait_controller PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/elkapod_gait_controller>
)
target_link_libraries(elkapod_gait_controller
        PUBLIC
        elkapod_gait_controller_parameters)
ament_target_dependencies(elkapod_gait_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface elkapod_gait_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
        DIRECTORY include/
        DESTINATION include/elkapod_gait_controller
)
install(TARGETS elkapod_gait_controller elkapod_gait_controller_parameters
        EXPORT export_elkapod_gait_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_targets(export_elkapod_gait_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

