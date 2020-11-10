cmake_minimum_required(VERSION 2.8.3)
project(robot_control)



add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    )


find_package(Boost REQUIRED system filesystem date_time thread)



catkin_package(
      INCLUDE_DIRS include
      LIBRARIES robot_control
      CATKIN_DEPENDS roscpp rospy moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
      DEPENDS  EIGEN3
    )


include_directories(
     include
    ${catkin_INCLUDE_DIRS}
    )



# add_executable(brick_wall_demo src/brick_wall_demo.cpp src/robot.cpp)

# target_link_libraries(brick_wall_demo
#    ${catkin_LIBRARIES}
#  )

#add_dependencies(brick_wall_demo ${brick_wall_demo_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(moveit_tutorial src/moveit_tutorial.cpp)


#target_link_libraries(moveit_tutorial
#   ${catkin_LIBRARIES}
# )

#add_dependencies(moveit_tutorial ${moveit_tutorial_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




