cmake_minimum_required(VERSION 2.8.3)

project(ur_gazebo)

find_package(catkin REQUIRED COMPONENTS
controller_manager
joint_state_controller
robot_state_publisher)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test.xml)
endif()

install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
