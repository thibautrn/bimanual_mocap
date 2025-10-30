cmake_minimum_required(VERSION 3.8)
project(bimanualrobot_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# Create executable
# add_executable(arm_loop_controller)
# ament_target_dependencies(arm_loop_controller rclcpp rclcpp_action control_msgs)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY scripts src
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/move_bimanual_drumsticks.py
  scripts/arm_loop_controller.py
  scripts/move_right_arm.py
  scripts/move_right_arm_pin_follow.py
  scripts/move_right_arm_pin.py
  scripts/move_right_arm_wrist.py
  scripts/move_right_arm2.py
  scripts/move_right_arm2.2.py
  scripts/move_right_arm2.2elb.py
  scripts/move_right_arm2.2elb_pick.py
  scripts/move_right_arm2.2elb_traj.py
  scripts/move_right_arm2.2elb_traj_copy.py 

  scripts/move_right_arm_horizon.py
  scripts/move_right_arm_fast.py
  scripts/move_right_arm_fast_dmp.py
  scripts/move_right_arm_fast_wrist.py
  scripts/move_right_arm_fast_record.py
  scripts/move_right_arm_fast_hz.py
  scripts/move_right_arm_pred_csv.py
  scripts/move_right_arm_record.py
  scripts/move_right_arm_pred.py
  scripts/move_right_arm_basic.py
  scripts/move_right_arm_base.py
  scripts/move_left_arm_basic.py
  scripts/servo_udp_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

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
