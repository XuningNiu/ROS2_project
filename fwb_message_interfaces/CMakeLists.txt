cmake_minimum_required(VERSION 3.8)
project(fwb_message_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Declare message files
set(msg_files
  "msg/Device.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Include directories
include_directories(
  include
  ${rosidl_default_runtime_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
)

# Install headers and libraries
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
