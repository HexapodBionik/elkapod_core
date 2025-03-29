cmake_minimum_required(VERSION 3.8)
project(elkapod_leg_control_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(include)
FILE(GLOB_RECURSE INCLUDE_FILES "include/elkapod_leg_control_tests/*.hpp")

add_executable(elkapod_control src/elkapod_control.cpp src/elkapod_leg_controller.cpp src/elkapod_leg_inverse_kinematics.cpp)

target_sources(elkapod_control PRIVATE ${INCLUDE_FILES})

target_include_directories(elkapod_control PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/elkapod_leg_control_tests
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


target_compile_features(elkapod_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        elkapod_control
        "rclcpp"
        "geometry_msgs"
        "yaml-cpp"
)

target_link_libraries(elkapod_control yaml-cpp)

# Install the executable
install(TARGETS elkapod_control
        DESTINATION lib/${PROJECT_NAME}
)

# Install the include directory
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

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

ament_package()