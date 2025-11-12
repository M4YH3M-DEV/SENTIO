cmake_minimum_required(VERSION 3.8)
project(sentio_msgs)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Declare ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Affect.msg"
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()