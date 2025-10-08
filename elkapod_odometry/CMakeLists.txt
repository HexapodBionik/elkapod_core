cmake_minimum_required(VERSION 3.20)
project(elkapod_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
include_directories(include)
FILE(GLOB_RECURSE INCLUDE_FILES "include/elkapod_odometry/*.hpp")


add_executable(elkapod_relay src/elkapod_touch_sensor_relay_init.cpp src/elkapod_touch_sensor_relay.cpp)
target_sources(elkapod_relay PRIVATE ${INCLUDE_FILES})

target_include_directories(elkapod_relay PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/elkapod_odometry
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


target_compile_features(elkapod_relay PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
        elkapod_relay
        "rclcpp"
        "geometry_msgs"
        "ros_gz_interfaces"
)

# Install the executable
install(TARGETS elkapod_relay
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(elkapod_odom src/elkapod_odom_init.cpp src/elkapod_odom.cpp src/elkapod_leg_kinematics.cpp)
target_sources(elkapod_odom PRIVATE ${INCLUDE_FILES})

target_include_directories(elkapod_odom PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/elkapod_odometry
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


target_compile_features(elkapod_odom PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
        elkapod_odom
        "rclcpp"
        "geometry_msgs"
        "ros_gz_interfaces"
        "sensor_msgs"
        "tf2"
        "tf2_geometry_msgs"
        "nav_msgs"
        "tf2_eigen"
        "tf2_ros"
)

# Install the executable
install(TARGETS elkapod_odom
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(imu_republisher src/imu_republisher.cpp src/imu_republisher_init.cpp)
target_sources(imu_republisher PRIVATE ${INCLUDE_FILES})

target_include_directories(imu_republisher PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/elkapod_odometry
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


target_compile_features(imu_republisher PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
        imu_republisher
        "rclcpp"
        "sensor_msgs"
)

# Install the executable
install(TARGETS imu_republisher
        DESTINATION lib/${PROJECT_NAME}
)


# Install the include directory
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
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