cmake_minimum_required(VERSION 3.8)
project(polarisutils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install examples and launch files
install(
  DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

# Install Python scripts if any
install(PROGRAMS
  # Add any executable Python scripts here
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_copyright REQUIRED)
  find_package(ament_flake8 REQUIRED)
  find_package(ament_pep257 REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs can_msgs)

ament_package()