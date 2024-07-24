cmake_minimum_required(VERSION 3.8)
project(fwb_msg_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(json-c REQUIRED)

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
  ${json-c_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
)

# Add executable
add_library(${PROJECT_NAME}_lib SHARED
  src/device_conversion.c
)

ament_target_dependencies(${PROJECT_NAME}_lib
  json-c
  rosidl_default_runtime
)

# Ensure device.h is generated before compiling device_conversion.c
add_dependencies(${PROJECT_NAME}_lib
  ${PROJECT_NAME}__rosidl_generator_c
)

# Install headers and libraries
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
