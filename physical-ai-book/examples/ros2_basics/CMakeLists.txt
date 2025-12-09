cmake_minimum_required(VERSION 3.8)
project(ros2_basics_examples)

# Check if running on Windows and adjust settings accordingly
if(WIN32)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  endif()
endif()

# Standard C++ settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create the talker executable
add_executable(talker talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Create the listener executable
add_executable(listener listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Install executables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/examples
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()