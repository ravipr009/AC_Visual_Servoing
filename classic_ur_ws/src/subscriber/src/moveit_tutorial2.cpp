
///*********************************************************************
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2013, SRI International
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of SRI International nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *********************************************************************/


//#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

//#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <cstdlib>
//#include <visualization_msgs/Marker.h>






//#include<sstream>
//#include <stdlib.h>
//#include <iostream>
//#include<string>


//#include<image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include<opencv2/core/core.hpp>
//#include<opencv2/imgproc/imgproc.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include<opencv2/opencv.hpp>
//#include<tf/transform_listener.h>
//#include<tf/tf.h>


//#include <sensor_msgs/image_encodings.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>

//#include<pcl_conversions/pcl_conversions.h>
//#include<pcl_ros/point_cloud.h>
//#include<sensor_msgs/PointCloud.h>
//#include<sensor_msgs/PointCloud2.h>
//#include<pcl_ros/pcl_nodelet.h>
//#include<pcl_ros/io/pcd_io.h>
//#include<pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <boost/foreach.hpp>

//#include<visualization_msgs/Marker.h>




//#include <moveit/planning_pipeline/planning_pipeline.h>
//#include <moveit/plan_execution/plan_execution.h>
//#include <moveit/plan_execution/plan_with_sensing.h>
//#include <moveit/trajectory_processing/trajectory_tools.h>
//#include <moveit/kinematic_constraints/utils.h>
//#include <moveit/move_group/capability_names.h>


//using namespace cv;
//namespace enc = sensor_msgs::image_encodings;
//cv::Mat img;

//unsigned int white_x=0;
//unsigned int white_y=0;


//using namespace std;

//bool flag_click = false;

//cv::Mat current_imageHSV;
//cv::Mat current_imageBIN;
//#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

//#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <cstdlib>
//#include <visualization_msgs/Marker.h>
//int thresh = 15;






//#define SPACE_KEY (32)

//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_point_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//void cloud_call_back(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
//{
//    pcl::PCLPointCloud2 pcl_pc2;
//    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
//    pcl::fromPCLPointCloud2(pcl_pc2,*current_point_cloud);
//    return;
//}


//cv::Mat current_image;
//void image_call_back(const sensor_msgs::ImageConstPtr &msg)
//{

//  current_image = cv_bridge::toCvShare(msg,"")->image ;

//   return;
//}



//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "move_group_interface_tutorial");
//  ros::NodeHandle node_handle;


//  ros::Subscriber image_sub = node_handle.subscribe <sensor_msgs::Image> ("/camera1/depth/image_raw", 1, image_call_back);
// ros::Subscriber cloud_sub = node_handle.subscribe <sensor_msgs::PointCloud2> ("/camera1/depth/points", 1, cloud_call_back);


//  geometry_msgs::PointStamped point_wrt_kinect;
//  geometry_msgs::PointStamped point_wrt_world;
//  tf::TransformListener listener;


// ros::Rate rate(10.0);
//  //ros::MultiThreadedSpinner spinner(4);
//  //spinner.spin();

// ros::AsyncSpinner spinner(4);
// spinner.start();


//  while (ros::ok())
//  {

////   while(current_image.rows == 0 )
////    {
////      ros::spinOnce();
////      rate.sleep();
////      //sleep(10.0);
////      std::cout<<"current_image.rows"<<current_image.rows<<endl;
////          }


//if(current_image.rows!=0)
//{
//    std::cout<<"current_image.rows"<<current_image.rows<<endl;
//  cv::imshow("current_image", current_image);
//      cv::waitKey(25);

//      //cv::Mat result(current_image.rows,current_image.cols, CV_8UC3, cv::Scalar(0,0,0));


//        char key2;
//        unsigned int i,j=0;
//        unsigned int t = 0;



//                unsigned int white_count =  0; ////Count of all white pixels

//                vector <unsigned int> centroid_x;
//                vector <unsigned int> centroid_y;

//                        white_x = 0;
//                        white_y = 0;
//                        white_count = 0;


//                        int filter[6] = {126, 0, 255, 172, 242, 0};
//                        cvtColor(current_image, current_imageHSV, CV_BGR2HSV); // Convert colour to HSV

//                      cv::inRange(current_imageHSV, cv::Scalar(filter[1], filter[3], filter[5]), cv::Scalar(filter[0], filter[2], filter[4]),current_imageBIN); //Filter HSV image to bet Buinary image

//                        imshow("Camera", current_image);
//                        imshow("Camera HSV", current_imageHSV);
//                        imshow("Camera BIN", current_imageBIN);

//                        for(unsigned int i=0; i<current_imageBIN.rows; i++)
//                                       {

//                                               for(unsigned int j=0; j<current_imageBIN.cols; j++)
//                                               {
//                                                       if(current_imageBIN.data[i*current_imageBIN.cols + j] > 200) // If colour is white
//                                                       {
//                                                               white_count++; // Number of white pixels
//                                                               white_x += j; //
//                                                               white_y += i; //
//                                                       }
//                                               }
//                                       }

//                                       if(white_count > thresh)
//                                       {
//                                               white_x /= white_count;
//                                               white_y /= white_count;
//                                               centroid_x.push_back(white_x);
//                                               centroid_y.push_back(white_y);
//                                       }
//                                       else
//                                       {
//                                               white_x = 0;
//                                               white_y = 0;
//                                               centroid_x.clear();
//                                               centroid_y.clear();
//                                         //    result = Scalar(0,0,0);
//                                       }



//                                       if(centroid_x.size() > 1)
//                                       {

//                                      // line(result, Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Scalar(255,255,255), 5,8);


//                                       }



//                                       circle(current_image, Point(white_x, white_y), 5, Scalar(0,255,2), 2, 8, 0);

//                                         cout<<"("<<white_x<<","<<white_y<<")"<<endl;





//    point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";

//    point_wrt_kinect.point.x = current_point_cloud->at(white_x, white_y).x;
//    point_wrt_kinect.point.y = current_point_cloud->at(white_x, white_y).y;
//    point_wrt_kinect.point.z = current_point_cloud->at(white_x, white_y).z;

//    listener.waitForTransform( "base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3));
//    listener.transformPoint("base_link", point_wrt_kinect, point_wrt_world);

//    std::cout << "point wrt_world -> x " << point_wrt_world.point.x
//              << ",  y " << point_wrt_world.point.y
//              << ", z " << point_wrt_world.point.z << std::endl;

//    std::cout << "point_wrt_kinect -> x " << point_wrt_kinect.point.x
//              << ",  y " << point_wrt_kinect.point.y
//              << ", z " << point_wrt_kinect.point.z << std::endl;


//    char a;

//       std::cout << "if satisfied y else n" << std::endl;
//       std::cin >> a;

//       if (a == 'y')
//         break;


//}


//   }



//  while(ros::ok())
//  {

//  std::cout<<"hello"<<endl;
//      static const std::string PLANNING_GROUP = "manipulator";
//      std::cout<<"hello"<<endl;
//      static const std::string PLANNER ="RRTConnectkConfigDefault";
//      std::cout<<"hello"<<endl;

//      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//      std::cout<<"hello"<<endl;
//      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//      std::cout<<"hello"<<endl;



////        const robot_state::JointModelGroup* joint_model_group =
////           move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



//  std::cout<<"hello"<<endl;
//          ros::Publisher display_publisher=node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);

//          moveit_msgs::DisplayTrajectory display_trajectory;

//            //Setting custom goal position
//            geometry_msgs::Pose target_pose1;
//std::cout<<"hello"<<endl;


//            target_pose1.orientation.w=1;
//            target_pose1.orientation.x=0;
//            target_pose1.orientation.y=0;
//            target_pose1.orientation.z=0;
//            target_pose1.position.x=point_wrt_world.point.x;
//            target_pose1.position.y=point_wrt_world.point.y;
//             target_pose1.position.z=point_wrt_world.point.z;


////            geometry_msgs::Pose target_pose1;
////            target_pose1.orientation.w=1;
////            target_pose1.orientation.x=0;
////            target_pose1.orientation.y=0;
////            target_pose1.orientation.z=0;

////            target_pose1.position.x=0.5;
////            target_pose1.position.y=0.1;
////            target_pose1.position.z=0.4;
//            move_group.setPoseTarget(target_pose1);


//              //Motion plan from currentlocation to custom position
//              moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//              bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//              /*Sleep to give Rviz time to visualize the plan*/
//              sleep(5.0);
// //             move_group.execute(my_plan);

//             move_group.move();

////std::cout<<"hello"<<endl;
////                //JOINT-SPACE GOAL

////                moveit::core::RobotStatePtr current_state=move_group.getCurrentState();
////                std::vector<double> joint_group_positions;
//////                current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);

////                //planning to the new joint space goal
////                joint_group_positions[0]=-1.0; //radians
////                move_group.setJointValueTarget(joint_group_positions);
////                success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
////                ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

////                 move_group.move();
////                 std::cout<<"hello"<<endl;


//ros::shutdown();
//}

//  std::cout<<"hello"<<endl;
//  return 0;
//}
////  ros::AsyncSpinner spinner(1);
////  spinner.start();

////  // BEGIN_TUTORIAL

////  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
////  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
////  // are used interchangably.
////  static const std::string PLANNING_GROUP = "manipulator";
////  static const std::string PLANNER ="RRTConnectkConfigDefault";


////  // The :move_group_interface:`MoveGroup` class can be easily
////  // setup using just the name of the planning group you would like to control and plan for.
////  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

////  // We will use the :planning_scene_interface:`PlanningSceneInterface`
////  // class to add and remove collision objects in our "virtual base_link" scene
////  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

////  // Raw pointers are frequently used to refer to the planning group for improved performance.
////  const robot_state::JointModelGroup* joint_model_group =
////      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

////  ros::Publisher display_publisher=node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
////  moveit_msgs::DisplayTrajectory display_trajectory;

////  //Setting custom goal position
////  geometry_msgs::Pose target_pose1;
////  target_pose1.orientation.w=1;
////  target_pose1.orientation.x=0;
////  target_pose1.orientation.y=0;
////  target_pose1.orientation.z=0;

////  target_pose1.position.x=0.5;
////  target_pose1.position.y=0.1;
////  target_pose1.position.z=0.4;
////  move_group.setPoseTarget(target_pose1);


////  //Motion plan from currentlocation to custom position
////  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
////  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

////  /*Sleep to give Rviz time to visualize the plan*/
////  sleep(5.0);

////  move_group.move();

////  //JOINT-SPACE GOAL

////  moveit::core::RobotStatePtr current_state=move_group.getCurrentState();
////  std::vector<double> joint_group_positions;
////  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
////  //planning to the new joint space goal
////  joint_group_positions[0]=-1.0; //radians
////  move_group.setJointValueTarget(joint_group_positions);
////  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
////  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

////   move_group.move();

//////   //planning with path constraints
//////   moveit_msgs::OrientationConstraint ocm;
//////   ocm.link_name="ee_link";
//////   ocm.header.frame_id="world";
//////   ocm.orientation.w=1.0;
//////   ocm.absolute_x_axis_tolerance=0.1;
//////   ocm.absolute_y_axis_tolerance=0.1;
//////   ocm.absolute_z_axis_tolerance=0.1;
//////   ocm.weight=1;

//////   moveit_msgs::Constraints test_constraints;
//////   test_constraints.orientation_constraints.push_back(ocm);
//////   move_group.setPathConstraints(test_constraints);
//////   //setting new start pose
//////   robot_state::RobotState start_state(*move_group.getCurrentState());
//////   geometry_msgs::Pose start_pose2;
//////   start_pose2.orientation.w=1.0;
//////start_pose2.orientation.x=0;
//////start_pose2.orientation.y=0;
//////start_pose2.orientation.z=0;
//////start_pose2.position.x=0.55;
//////start_pose2.position.y=-0.05;
//////start_pose2.position.z=0.8;
//////start_state.setFromIK(joint_model_group,start_pose2);
//////move_group.setStartState(start_state);
//////planning to the earlier pose target
//////move_group.setPoseTarget(target_pose1);
//////increasing planning time to 10 sec
//////move_group.setPlanningTime(10.0);

//////success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//////ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

////////move
//////move_group.move();
////   ////clearing path constraints
////   //move_group.clearPathConstraints();







////  ros::shutdown();
////  return 0;
////}


//#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

//#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <cstdlib>
//#include <visualization_msgs/Marker.h>






#include<sstream>
#include <stdlib.h>
#include <iostream>
#include<string>


#include<image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<tf/transform_listener.h>
#include<tf/tf.h>


#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/pcl_nodelet.h>
#include<pcl_ros/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/foreach.hpp>

#include<visualization_msgs/Marker.h>




#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>



#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef pcl::PointXYZRGB PointType;


int k = 0;

bool flag_img = false;
bool flag_cloud = false;
bool flag_click = false;

int clk_row, clk_col;


using namespace cv;
namespace enc = sensor_msgs::image_encodings;
cv::Mat img;

unsigned int white_x=0;
unsigned int white_y=0;


using namespace std;
int thresh=15;


cv::Mat current_imageHSV;
cv::Mat current_imageBIN;
cv::Mat current_image;


void image_call_back(const sensor_msgs::ImageConstPtr &msg)
{

  if(!flag_img)
  {
    std::cout << "\nim in imageCallback";
//    current_image = cv_bridge::toCvShare(msg, "")->image ;
    current_image = cv_bridge::toCvShare(msg, "bgr8")->image ;

    k = 1;
    flag_img = true;
    usleep(10000);
  }


  //  return;
}


pcl::PointCloud<PointType>::Ptr current_point_cloud (new pcl::PointCloud<PointType>);
void cloud_call_back (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  if(!flag_cloud)
  {
    std::cout << "\nim in cloud_call_back";

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*current_point_cloud);
    k = 2;
    flag_cloud = true;
    usleep(10000);
  }
}


//void CallBackFunc(int event, int x, int y, int flags, void* userdata)
//{

//  std::cout << "In call back with flag_click " << flag_click << std::endl;
//  if(!flag_click)
//  {


//    if  ( event == cv::EVENT_LBUTTONDOWN )
//    {
//      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;

//      clk_row = y;
//      clk_col = x;

//      if ( std::isnan(current_point_cloud->at(x, y).x))
//      {
//        std::cout << "Nan detected " << std::endl;
//      }
//      else
//      {
//        flag_click = 1;
//      }
//    }

//  }




//}




int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_transformation");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image> ("camera1/depth/image_raw", 2, image_call_back);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("/camera1/depth/points", 1, cloud_call_back);


  spinner.start();


  while(!flag_cloud | !flag_img | (current_point_cloud->height == 0) | (current_point_cloud->width == 0) )
  {
    usleep(100000);



//    char a;
//    std::cout << "satisfied y or n" << std::endl;
//    std::cin >> a;

//    if (a == 'y')
      break;
//    else
//    {
//      flag_cloud = false;
//      flag_img = false;

//    }

  }




  //    spinner.stop();

  //    std::cout << "\n\nboth spinners have been stopped\n";
  std::cout <<"\nk: " << k;


  moveit::planning_interface::MoveGroup group("manipulator");
  static const std::string PLANNER ="RRTConnectkConfigDefault";

  geometry_msgs::PointStamped point_wrt_kinect;
  geometry_msgs::PointStamped point_wrt_world;
  tf::TransformListener listener;







  cv::namedWindow("view");
  cv::startWindowThread();
  //cv::setMouseCallback("view", CallBackFunc, NULL);




  while(!flag_click)
  {
    cv::imshow("view", current_image);

    cv::waitKey(30);
    cv::Mat result(current_image.rows,current_image.cols, CV_8UC3, cv::Scalar(0,0,0));


      char key2;
      unsigned int i,j=0;
      unsigned int t = 0;



              unsigned int white_count =  0; ////Count of all white pixels

              vector <unsigned int> centroid_x;
              vector <unsigned int> centroid_y;

                      white_x = 0;
                      white_y = 0;
                      white_count = 0;


                      int filter[6] = {255,0,255,0,255, 216};
                      cvtColor(current_image, current_imageHSV, CV_BGR2HSV); // Convert colour to HSV

                    cv::inRange(current_imageHSV, cv::Scalar(filter[1], filter[3], filter[5]), cv::Scalar(filter[0], filter[2], filter[4]),current_imageBIN); //Filter HSV image to bet Buinary image

                      imshow("Camera", current_image);
                      imshow("Camera HSV", current_imageHSV);
                      imshow("Camera BIN", current_imageBIN);

                      for(unsigned int i=0; i<current_imageBIN.rows; i++)
                                     {
                                             for(unsigned int j=0; j<current_imageBIN.cols; j++)
                                             {
                                                     if(current_imageBIN.data[i*current_imageBIN.cols + j] > 200) // If colour is white
                                                     {
                                                             white_count++; // Number of white pixels
                                                             white_x += j; //
                                                             white_y += i; //
                                                     }
                                             }
                                     }

                                     if(white_count > thresh)
                                     {
                                             white_x /= white_count;
                                             white_y /= white_count;
                                             centroid_x.push_back(white_x);
                                             centroid_y.push_back(white_y);
                                     }
                                     else
                                     {
                                             white_x = 0;
                                             white_y = 0;
                                             centroid_x.clear();
                                             centroid_y.clear();
                                           result = Scalar(0,0,0);
                                     }



                                     if(centroid_x.size() > 1)
                                     {

                                     line(result, Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Scalar(255,255,255), 5,8);


                                     }



                                     circle(current_image, Point(white_x, white_y), 5, Scalar(0,255,2), 2, 8, 0);

                                       cout<<"("<<white_x<<","<<white_y<<")"<<endl;





  point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";

  point_wrt_kinect.point.x = current_point_cloud->at(white_x, white_y).x;
  point_wrt_kinect.point.y = current_point_cloud->at(white_x, white_y).y;
  point_wrt_kinect.point.z = current_point_cloud->at(white_x, white_y).z;

  listener.waitForTransform( "base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3));
  listener.transformPoint("base_link", point_wrt_kinect, point_wrt_world);

  std::cout << "point wrt_world -> x " << point_wrt_world.point.x
            << ",  y " << point_wrt_world.point.y
            << ", z " << point_wrt_world.point.z << std::endl;

  std::cout << "point_wrt_kinect -> x " << point_wrt_kinect.point.x
            << ",  y " << point_wrt_kinect.point.y
            << ", z " << point_wrt_kinect.point.z << std::endl;


  char a;

     std::cout << "if satisfied y else n" << std::endl;
     std::cin >> a;

     if (a == 'y')
       break;
}







//  point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";

//  point_wrt_kinect.point.x = current_point_cloud->at(clk_col, clk_row).x;
//  point_wrt_kinect.point.y = current_point_cloud->at(clk_col, clk_row).y;
//  point_wrt_kinect.point.z = current_point_cloud->at(clk_col, clk_row).z;

//  listener.waitForTransform( "base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3));
//  listener.transformPoint("base_link", point_wrt_kinect, point_wrt_world);




  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_test_marker", 10);


  while(ros::ok())
  {

    std::cout << "See RVIZ" << std::endl;
    visualization_msgs::Marker centroid_point;
    centroid_point.header.frame_id = "/world";
    centroid_point.header.stamp = ros::Time::now();
    centroid_point.ns = "test_points";
    centroid_point.action = visualization_msgs::Marker::ADD;
    centroid_point.pose.orientation.w  = 1.0;


    centroid_point.id = 0;



    centroid_point.type = visualization_msgs::Marker::POINTS;



    centroid_point.scale.x = 0.2;
    centroid_point.scale.y = 0.2;

    centroid_point.color.r = 1.0f;
    centroid_point.color.a = 1.0;


//       point_wrt_world.point.x= 0.719122;
//       point_wrt_world.point.y=0.212353;
//       point_wrt_world.point.z= -0.265968;

    geometry_msgs::Point p_c;
    p_c = point_wrt_world.point;
    centroid_point.points.push_back(p_c);

    std::cout << "point wrt_world -> x " << point_wrt_world.point.x
              << ",  y " << point_wrt_world.point.y
              << ", z " << point_wrt_world.point.z << std::endl;

    std::cout << "point_wrt_kinect -> x " << point_wrt_kinect.point.x
              << ",  y " << point_wrt_kinect.point.y
              << ", z " << point_wrt_kinect.point.z << std::endl;



    marker_pub.publish(centroid_point);



    geometry_msgs::PoseStamped current_Pose = group.getCurrentPose();

    geometry_msgs::Pose target_pose;

    target_pose.position = point_wrt_world.point;
    target_pose.orientation = current_Pose.pose.orientation;


    group.setPoseTarget(target_pose);

    group.move();

  }







  return 0;

}

