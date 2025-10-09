cmake_minimum_required(VERSION 3.20)
project(elkapod_core_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
FILE(GLOB_RECURSE SRC "src/*.cpp")

add_library(elkapod_core_lib STATIC ${SRC})
target_compile_features(elkapod_core_lib PUBLIC c_std_99 cxx_std_20)
set_target_properties(elkapod_core_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(elkapod_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS elkapod_core_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
