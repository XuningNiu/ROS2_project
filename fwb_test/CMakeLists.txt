cmake_minimum_required(VERSION 3.12)

project(fwb_test)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(example_interfaces REQUIRED)
# find_package(rcl REQUIRED)
# find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(rcpputils REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fwb_exec src/fwb_pub.cpp)

# Specify dependencies for the target
ament_target_dependencies(fwb_exec rclcpp std_msgs)

# Install the executable
install(TARGETS
  fwb_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
