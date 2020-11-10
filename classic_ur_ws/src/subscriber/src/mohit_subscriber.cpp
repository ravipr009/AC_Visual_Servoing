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


//namespace enc = sensor_msgs::image_encodings;


//    int clicked_x,  clicked_y;


//cv::Mat img;
//int k = 0;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZRGB>);

//char a;


//bool flag_img = false;
//bool flag_cloud = false;
//bool flag_click = false;

//int number_of_points = 1;
//int number_of_points_selected = 0;


//cv::Mat current_image;
//void image_call_back(const sensor_msgs::ImageConstPtr &msg)
//{

//  current_image = cv_bridge::toCvShare(msg, "bgr8")->image ;

//   return;
//}


//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_point_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//void cloud_call_back (const sensor_msgs::PointCloud2ConstPtr& msg)
//{
//  pcl::PCLPointCloud2 pcl_pc2;
//  pcl_conversions::toPCL(*msg, pcl_pc2);
//  pcl::fromPCLPointCloud2(pcl_pc2,*current_point_cloud);
//  return;
//}




//void CallBackFunc(int event, int x, int y, int flags, void* userdata)
//{

//  std::cout << "In call back with flag_click " << flag_click << std::endl;
//  if(!flag_click)
//  {


//    if  ( event == cv::EVENT_LBUTTONDOWN )
//    {
//      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;


//      clicked_y = y;
//      clicked_x = x;

//      std::cout << "current_point_cloud " << current_point_cloud->height << std::endl;

//      if ( std::isnan(current_point_cloud->at(x, y).x))
//      {
//        std::cout << "Nan detected " << std::endl;

//      }
//      else
//      {
//        number_of_points_selected = number_of_points_selected + 1;

//        if (number_of_points_selected == number_of_points)
//          flag_click = 1;
//      }
//    }

//  }




//}








//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "image_listener");
//  ros::NodeHandle n;
//  ros::Subscriber image_sub = n.subscribe <sensor_msgs::Image> ("/camera1/depth/image_raw", 1, image_call_back);
//  ros::Subscriber cloud_sub = n.subscribe <sensor_msgs::PointCloud2> ("/camera1/depth/points", 1, cloud_call_back);


//  geometry_msgs::PointStamped point_wrt_kinect;
//  geometry_msgs::PointStamped point_wrt_world;
//  tf::TransformListener listener;


//  ros::Rate rate(10.0);






//  while (ros::ok())
//  {

//    while(current_image.rows == 0 )
//    {
//      ros::spinOnce();
//      rate.sleep();
//    }







//   std::cout << "current_point_cloud->height " << current_point_cloud->height << std::endl;


//    cv::namedWindow("current_image");
//    cv::startWindowThread();
//    cv::setMouseCallback("current_image", CallBackFunc, NULL);

//    while(!flag_click)
//    {
//      cv::imshow("current_image", current_image);
//      std::cout << "hi" << std::endl;
//      cv::waitKey(25);
//    }


//    char a;

//    std::cout << "satisfied y on n" << std::endl;
//    std::cin >> a;

//    if (a == 'y')
//      break;

//    else
//      flag_click = false;


//    std::cout << "image" << std::endl;


//}






//  point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";

//  point_wrt_kinect.point.x = current_point_cloud->at(clicked_x, clicked_y).x;
//  point_wrt_kinect.point.y = current_point_cloud->at(clicked_x, clicked_y).y;
//  point_wrt_kinect.point.z = current_point_cloud->at(clicked_x, clicked_y).z;

//  listener.waitForTransform( "base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3));
//  listener.transformPoint("base_link", point_wrt_kinect, point_wrt_world);

//  std::cout << "point wrt_world -> x " << point_wrt_world.point.x
//            << ",  y " << point_wrt_world.point.y
//            << ", z " << point_wrt_world.point.z << std::endl;

//  std::cout << "point_wrt_kinect -> x " << point_wrt_kinect.point.x
//            << ",  y " << point_wrt_kinect.point.y
//            << ", z " << point_wrt_kinect.point.z << std::endl;




//  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_test_marker", 10);





//  while(ros::ok())
//  {

//    std::cout << "See RVIZ" << std::endl;
//    visualization_msgs::Marker centroid_point;
//    centroid_point.header.frame_id = "/base_link";
//    centroid_point.header.stamp = ros::Time::now();
//    centroid_point.ns = "test_points";
//    centroid_point.action = visualization_msgs::Marker::ADD;
//    centroid_point.pose.orientation.w  = 1.0;


//    centroid_point.id = 0;



//    centroid_point.type = visualization_msgs::Marker::POINTS;



//    centroid_point.scale.x = 0.02;
//    centroid_point.scale.y = 0.02;

//    centroid_point.color.b = 1.0f;
//    centroid_point.color.a = 1.0;


//    for (int i=0 ;i<number_of_points; i++)
//    {
//      geometry_msgs::Point p_c;
//      p_c = point_wrt_world.point;
//      centroid_point.points.push_back(p_c);

//      std::cout << "point wrt_world -> x " << point_wrt_world.point.x
//                << ",  y " << point_wrt_world.point.y
//                << ", z " << point_wrt_world.point.z << std::endl;

//      std::cout << "point_wrt_kinect -> x " << point_wrt_kinect.point.x
//                << ",  y " << point_wrt_kinect.point.y
//                << ", z " << point_wrt_kinect.point.z << std::endl;

//    }

//    marker_pub.publish(centroid_point);

//  }







//  return 0;
//}


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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef pcl::PointXYZRGB PointType;


int k = 0;

bool flag_img = false;
bool flag_cloud = false;
bool flag_click = false;

int clk_row, clk_col;







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


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{

  std::cout << "In call back with flag_click " << flag_click << std::endl;
  if(!flag_click)
  {


    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;

      clk_row = y;
      clk_col = x;

      if ( std::isnan(current_point_cloud->at(x, y).x))
      {
        std::cout << "Nan detected " << std::endl;
      }
      else
      {
        flag_click = 1;
      }
    }

  }




}




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


  moveit::planning_interface::MoveGroupInterface group("manipulator");

  geometry_msgs::PointStamped point_wrt_kinect;
  geometry_msgs::PointStamped point_wrt_world;
  tf::TransformListener listener;







  cv::namedWindow("view");
  cv::startWindowThread();
  cv::setMouseCallback("view", CallBackFunc, NULL);




  while(!flag_click)
  {
    cv::imshow("view", current_image);
    cv::waitKey(30);
  }




  point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";

  point_wrt_kinect.point.x = current_point_cloud->at(clk_col, clk_row).x;
  point_wrt_kinect.point.y = current_point_cloud->at(clk_col, clk_row).y;
  point_wrt_kinect.point.z = current_point_cloud->at(clk_col, clk_row).z;

  listener.waitForTransform( "base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3));
  listener.transformPoint("base_link", point_wrt_kinect, point_wrt_world);




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



    centroid_point.scale.x = 0.02;
    centroid_point.scale.y = 0.02;

    centroid_point.color.r = 1.0f;
    centroid_point.color.a = 1.0;



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

//    geometry_msgs::Pose target_pose;

//    target_pose.position = point_wrt_world.point;
//    target_pose.orientation = current_Pose.pose.orientation;


//    geometry_msgs::PoseStamped robot_pose;
//      robot_pose = group.getCurrentPose();

//      geometry_msgs::Pose current_position;
//      current_position = robot_pose.pose;
//      /*Retrive position and orientation */
//        geometry_msgs::Point exact_pose = current_position.position;
//        geometry_msgs::Quaternion exact_orientation = current_position.orientation;

//        ROS_INFO("Reference frame : %s",group.getPlanningFrame().c_str());
//        ROS_INFO("Reference frame : %s",group.getEndEffectorLink().c_str());


//        std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
//        std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;
    geometry_msgs::Pose target_pose;

    target_pose.position = point_wrt_world.point;
    target_pose.orientation = current_Pose.pose.orientation;
    group.setGoalPositionTolerance(0.1);
    group.setGoalOrientationTolerance(0.1);

    group.setMaxVelocityScalingFactor(1.0);
    group.setGoalTolerance(0.001);

    std::vector<geometry_msgs::Pose> waypoints;


    waypoints.push_back(target_pose);  // down and right (back to start)

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
          fraction * 100.0);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(15.0);

//    group.setPoseTarget (target_pose);




    group.execute();

  }







  return 0;

}
