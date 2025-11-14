cmake_minimum_required(VERSION 3.8)
project(bimanualrobot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_image REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)

# Find NEW Gazebo (gz-sim) instead of old gazebo
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)

# Build the ball leash plugin
add_library(ball_leash_plugin SHARED src/ball_leash_plugin.cc)
target_include_directories(ball_leash_plugin PUBLIC 
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
  ${gz-common5_INCLUDE_DIRS}
)
target_link_libraries(ball_leash_plugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
)

# Install the plugin library
install(TARGETS ball_leash_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Copy necessary files to designated locations in the project 
install(
  DIRECTORY config launch models worlds scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Copy necessary files to designated locations in the project
install(PROGRAMS 
  scripts/spawn_drum.py
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()