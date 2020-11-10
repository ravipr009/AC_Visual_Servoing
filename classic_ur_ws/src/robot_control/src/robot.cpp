#include <iostream>
#include <robot_control/brick_wall_demo.h>
#include <pthread.h>



robot::robot():
    nh_("~"),
    planning_group_name_("arm"),
    planner_name_("RRTConnectkConfigDefault"),
    POS_TOLARENCE(0.05),
    ANG_TOLARENCE(0.2),
    PLANING_TIME(20.0)
{
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlannerId(planner_name_);
    move_group_->setPlanningTime(PLANING_TIME);
    move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
    move_group_->setGoalPositionTolerance(POS_TOLARENCE);
    pose_of_cube_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_of_cube_topic",100);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("normal_point",1);
    move_group_->setGoalJointTolerance(0.05);

    sleep(5);
}




bool robot::move_to_joint_goal(std::vector<double> joint_goal, float speed_factor)
{

    move_group_->setJointValueTarget(joint_goal);

    move_group_->setMaxAccelerationScalingFactor(speed_factor);


    move_group_->setMaxVelocityScalingFactor(speed_factor);


    moveit::planning_interface::MoveGroup::Plan joint_plan;


    bool success = move_group_->plan(joint_plan);

    std::cout<<"success : "<<success<<"\n";

    if(success)
    {
        ROS_INFO(" Execute called");

//        char p;
//        p ='p';

//        ROS_INFO ("Press p to run");
//        std::cin>>p;

        move_group_->execute(joint_plan);

        ROS_INFO(" Execute finished");

        return true;
    }
    else
        return false;

    //    return (joint_goal_status(joint_goal));

}

bool robot::joint_goal_status(std::vector<double> joint_goal)
{
    std::vector<double> current_joint_angles = move_group_->getCurrentJointValues();

    float joint_threshold = 0.1;

    float joint_error = 0;

    for(int i = 0; i<6; i++)
        joint_error = joint_error + std::pow((current_joint_angles.at(i) - joint_goal.at(i)), 2);

    if(std::sqrt(joint_error) < joint_threshold)
        return true;z
    else
        return false;

}

bool robot::compute_cartesian_path(geometry_msgs::Pose pose_goal, float speed_factor)
{
    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan pose_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(normal);

    waypoints.push_back(pose_goal);

    double fraction = move_group_->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    std::cout << "\n Path Computed: " << 100*fraction <<"\n";
    if(fraction > 0.95)
    {
        robot_trajectory::RobotTrajectory rt (move_group_->getCurrentState()->getRobotModel(), "arm");

        rt.setRobotTrajectoryMsg(move_group_->getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        iptp.computeTimeStamps(rt, speed_factor, speed_factor);

        rt.getRobotTrajectoryMsg(trajectory);

        pose_plan.trajectory_ = trajectory;

//        std::cout<< "press to run \n";
//        char a;
//        a='a';
//        std::cin>>a;

        move_group_->execute(pose_plan);

        return true;
        //        return pose_goal_status(pose_goal);
    }
    return false;
}

bool robot::pose_goal_status(geometry_msgs::Pose pose_goal)
{
    geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();

    float linear_threshold = 0.01;

    float linear_error =  std::pow((current_pose.pose.position.x - pose_goal.position.x), 2) +
            std::pow((current_pose.pose.position.y - pose_goal.position.y), 2) +
            std::pow((current_pose.pose.position.z - pose_goal.position.z), 2);

    if(std::sqrt(linear_error) < linear_threshold)
        return true;
    else
        return false;

}







