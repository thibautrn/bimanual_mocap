# Install script for directory: /home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE DIRECTORY FILES
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/src"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bimanualrobot_system_tests" TYPE PROGRAM FILES
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_bimanual_drumsticks.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/arm_loop_controller.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_pin_follow.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_pin.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_wrist.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm2.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm2.2.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm2.2elb.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm2.2elb_pick.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm2.2elb_traj.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm2.2elb_traj_copy.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_horizon.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_fast.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_fast_dmp.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_fast_dmp_ball.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_fast_wrist.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_fast_record.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_fast_hz.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_pred_csv.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_record.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_pred.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_basic.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_right_arm_base.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/move_left_arm_basic.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/servo_udp_bridge.py"
    "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/robot_right_arm_record.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/bimanualrobot_system_tests")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/bimanualrobot_system_tests")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests/environment" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests/environment" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_index/share/ament_index/resource_index/packages/bimanualrobot_system_tests")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests/cmake" TYPE FILE FILES
    "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_core/bimanualrobot_system_testsConfig.cmake"
    "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/ament_cmake_core/bimanualrobot_system_testsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bimanualrobot_system_tests" TYPE FILE FILES "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/thibaut/Documents/bimanual_mocap/build/bimanualrobot_system_tests/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
