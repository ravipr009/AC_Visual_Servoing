#include<sstream>
#include <stdlib.h>
#include <iostream>
#include<string>

#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit/move_group_interface/move_group.h>
#include<moveit/planning_interface/planning_interface.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<moveit/robot_trajectory/robot_trajectory.h>
#include<moveit/trajectory_processing/iterative_time_parameterization.h>
#include<std_msgs/Float64MultiArray.h>
#include<cstdlib>




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




using namespace cv;
namespace enc = sensor_msgs::image_encodings;
cv::Mat img;

unsigned int white_x=0;
unsigned int white_y=0;


using namespace std;

bool flag_click = false;

cv::Mat current_imageHSV;
cv::Mat current_imageBIN;

int thresh = 15;






#define SPACE_KEY (32)

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_point_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
void cloud_call_back(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*current_point_cloud);
    return;
}


cv::Mat current_image;
void image_call_back(const sensor_msgs::ImageConstPtr &msg)
{

  current_image = cv_bridge::toCvShare(msg,"bgr8")->image ;

   return;
}



// We will use the :planning_scene_interface:`PlanningSceneInterface`
// class to add and remove collision objects in our "virtual base_link" scene



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  ros::Subscriber image_sub = n.subscribe <sensor_msgs::Image> ("/camera1/depth/image_raw", 1, image_call_back);
 ros::Subscriber cloud_sub = n.subscribe <sensor_msgs::PointCloud2> ("/camera1/depth/points", 1, cloud_call_back);


  geometry_msgs::PointStamped point_wrt_kinect;
  geometry_msgs::PointStamped point_wrt_world;
  tf::TransformListener listener;


  ros::Rate rate(10.0);






  while (ros::ok())
  {

   while(current_image.rows == 0 )
    {
      ros::spinOnce();
      rate.sleep();
          }




      cv::imshow("current_image", current_image);
      cv::waitKey(25);

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


                        int filter[6] = {126, 0, 255, 172, 242, 0};
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




//    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_test_marker", 10);






//     int number_of_points=1;
//      std::cout << "See RVIZ" << std::endl;
//      visualization_msgs::Marker centroid_point;
//      centroid_point.header.frame_id = "/base_link";
//      centroid_point.header.stamp = ros::Time::now();
//      centroid_point.ns = "test_points";
//      centroid_point.action = visualization_msgs::Marker::ADD;
//      centroid_point.pose.orientation.w  = 1.0;


//      centroid_point.id = 0;



//      centroid_point.type = visualization_msgs::Marker::POINTS;



//      centroid_point.scale.x = 0.02;
//      centroid_point.scale.y = 0.02;

//      centroid_point.color.b = 1.0f;
//      centroid_point.color.a = 1.0;


//      for (int i=0 ;i<number_of_points; i++)
//      {
//        geometry_msgs::Point p_c;
//        p_c = point_wrt_world.point;
//        centroid_point.points.push_back(p_c);

//        std::cout << "point wrt_world -> x " << point_wrt_world.point.x
//                  << ",  y " << point_wrt_world.point.y
//                  << ", z " << point_wrt_world.point.z << std::endl;

//        std::cout << "point_wrt_kinect -> x " << point_wrt_kinect.point.x
//                  << ",  y " << point_wrt_kinect.point.y
//                  << ", z " << point_wrt_kinect.point.z << std::endl;

//      }

//      marker_pub.publish(centroid_point);





return 0;
}


