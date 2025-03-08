cmake_minimum_required(VERSION 3.8)
project(mobility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-class-memaccess)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Manually locate tinyxml2 (since it doesn't provide a CMake config)
find_library(TINYXML2_LIB tinyxml2)
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h PATHS /usr/include /usr/local/include)

if (TINYXML2_LIB AND TINYXML2_INCLUDE_DIR)
    message(STATUS "Found TinyXML2: ${TINYXML2_LIB}, ${TINYXML2_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "TinyXML2 library not found!")
endif()

# add include this directory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${TINYXML2_INCLUDE_DIRS}
)

add_executable(mobility_node src/mobility_node.cpp)

target_link_libraries(mobility_node
  ${TINYXML2_LIB}
)

ament_target_dependencies(mobility_node rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  mobility_node
  DESTINATION lib/${PROJECT_NAME}
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
