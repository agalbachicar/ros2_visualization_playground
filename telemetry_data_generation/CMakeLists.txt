cmake_minimum_required(VERSION 3.5)
project(telemetry_data_generation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(telemetry_data_generation
  SHARED
    src/telemetry_data_generation/telemetry_data_generation.cpp
)
target_include_directories(telemetry_data_generation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(telemetry_data_generation
  builtin_interfaces
  geometry_msgs
  rclcpp
  std_msgs
)
install(
  TARGETS telemetry_data_generation
  EXPORT export_telemetry_data_generation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(telemetry_data_node
  src/telemetry_data_generation/telemetry_data_node.cpp
)
target_include_directories(telemetry_data_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(telemetry_data_node
  telemetry_data_generation
)
ament_target_dependencies(telemetry_data_node
  rclcpp
  geometry_msgs
)
install(
  TARGETS telemetry_data_node
  EXPORT export_telemetry_data_generation
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
