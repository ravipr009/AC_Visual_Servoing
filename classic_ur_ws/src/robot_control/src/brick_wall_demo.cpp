#include <iostream>
#include <stdio.h>
#include <robot_control/brick_wall_demo.h>
#include <signal.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>

#define speed 0.2

void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << "\n";

    exit(0);
}




char au;

void *parallel_thread(void *threadid)
{
    long tid;
    tid = (long)threadid;
    while(1)
    {
        std::cout << "press a, b, c, d, e, f, g, h  " << std::endl;
        std::cin >> au;
    }
}



int main(int argc, char** argv)
{

    ros::init( argc, argv,"brick_wall_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot ur5;

    geometry_msgs::PoseStamped pose_of_cube;


    ROS_INFO("MBZIRC Started :  Demo 1");

    std::vector<double> home_pos;
    home_pos.push_back(0.0232);
    home_pos.push_back(-1.611811);
    home_pos.push_back(1.5355);
    home_pos.push_back(-1.513);
    home_pos.push_back(-1.57);
    home_pos.push_back(0.0);


    std::vector<double> start_pos;
    start_pos.push_back(0.0);
    start_pos.push_back(-1.57);
    start_pos.push_back(1.57);
    start_pos.push_back(-1.57);
    start_pos.push_back(-1.57);
    start_pos.push_back(0.0);

    ROS_INFO("Home and start pos initialized");

    pthread_t threads[1];
    pthread_create(&threads[0], NULL, parallel_thread, (void *)1);

    bool success ;


    while(ros::ok())
    {
        signal(SIGINT, signal_callback_handler);


        ROS_INFO("A");

        success = ur5.move_to_joint_goal(home_pos,speed);
        if(success)
        {
            ROS_INFO("Home position reached");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(1000000);





        ROS_INFO("D");


        success = ur5.compute_cartesian_path(ur5.centroid,speed);
        if(success)
        {
            ROS_INFO("Normal position reached");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);


        ROS_INFO("E");

        success = ur5.compute_cartesian_path(ur5.lift,speed);
        if(success)
        {
            ROS_INFO("Lift position reached");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");
        usleep(1000000);


        ROS_INFO("A");

        success = ur5.move_to_joint_goal(home_pos,speed);
        if(success)
        {
            ROS_INFO("Home position reached");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");
        usleep(1000000);







        ROS_INFO("G");


        success = ur5.compute_cartesian_path(ur5.centroid,speed);
        if(success)
        {
            ROS_INFO("Normal position reached");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);




    }



    return 0;

}
