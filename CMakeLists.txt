cmake_minimum_required(VERSION 3.8)
project(botdylan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(trajectory src/botdylan/botdylan/trajectory.py) 
ament_target_dependencies(trajectory rclcpp std_msgs)  
# Install the executable
install(
  TARGETS trajectory
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and rviz files
install(
  DIRECTORY launch/ rviz/
  DESTINATION share/${PROJECT_NAME}
)

# Optional: Install Python files (if you have any)
# install(
#   PROGRAMS scripts/your_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Test and linting configuration (optional)
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
