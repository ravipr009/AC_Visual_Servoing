
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <std_msgs/Float64MultiArray.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL

  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string PLANNER ="RRTConnectkConfigDefault";


  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual base_link" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ros::Publisher display_publisher=node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  //Setting custom goal position
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w=1;
  target_pose1.orientation.x=0;
  target_pose1.orientation.y=0;
  target_pose1.orientation.z=0;

  target_pose1.position.x=0.5;
  target_pose1.position.y=0.1;
  target_pose1.position.z=0.4;
  move_group.setPoseTarget(target_pose1);


  //Motion plan from currentlocation to custom position
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  /*Sleep to give Rviz time to visualize the plan*/
  sleep(5.0);

  move_group.move();

  //JOINT-SPACE GOAL

  moveit::core::RobotStatePtr current_state=move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
  //planning to the new joint space goal
  joint_group_positions[0]=-1.0; //radians
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

   move_group.move();

//   //planning with path constraints
//   moveit_msgs::OrientationConstraint ocm;
//   ocm.link_name="ee_link";
//   ocm.header.frame_id="world";
//   ocm.orientation.w=1.0;
//   ocm.absolute_x_axis_tolerance=0.1;
//   ocm.absolute_y_axis_tolerance=0.1;
//   ocm.absolute_z_axis_tolerance=0.1;
//   ocm.weight=1;

//   moveit_msgs::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group.setPathConstraints(test_constraints);
//   //setting new start pose
//   robot_state::RobotState start_state(*move_group.getCurrentState());
//   geometry_msgs::Pose start_pose2;
//   start_pose2.orientation.w=1.0;
//start_pose2.orientation.x=0;
//start_pose2.orientation.y=0;
//start_pose2.orientation.z=0;
//start_pose2.position.x=0.55;
//start_pose2.position.y=-0.05;
//start_pose2.position.z=0.8;
//start_state.setFromIK(joint_model_group,start_pose2);
//move_group.setStartState(start_state);
//planning to the earlier pose target
//move_group.setPoseTarget(target_pose1);
//increasing planning time to 10 sec
//move_group.setPlanningTime(10.0);

//success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

////move
//move_group.move();
   ////clearing path constraints
   //move_group.clearPathConstraints();







  ros::shutdown();
  return 0;
}

