#ifndef __BRICK_WALL_DEMO_H_
#define __BRICK_WALL_DEMO_H_



#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <std_msgs/Float64MultiArray.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>

class robot
{
public:
    robot();
    ros::NodeHandle nh_;

    boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;


    std::string planning_group_name_;
    std::string planner_name_;
    double POS_TOLARENCE, ANG_TOLARENCE, PLANING_TIME;

    ros::ServiceClient get_pose_client;




    ros::Publisher pose_of_cube_pub;

    ros::Publisher marker_pub;


    geometry_msgs::Pose normal;
    geometry_msgs::Pose centroid;
    geometry_msgs::Pose lift;





    bool move_to_joint_goal(std::vector<double>joint_goal,float speed_factor);
    bool joint_goal_status (std::vector<double>joint_goal);
    bool compute_cartesian_path (geometry_msgs::Pose pose_goal, float speed_factor);
    bool pose_goal_status (geometry_msgs::Pose pose_goal);


};































#endif //__BRICK_WALL_DEMO_H_
