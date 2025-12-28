cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED) # Required for Actions

# --- DEFINE YOUR INTERFACES ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripCommand.srv"
  "action/MoveArm.action"
  DEPENDENCIES action_msgs
)

ament_package()