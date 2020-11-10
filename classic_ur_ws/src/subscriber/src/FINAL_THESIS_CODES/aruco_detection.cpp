//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/core.hpp"
//#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"

//#include "opencv2/opencv.hpp"
//#include "opencv2/calib3d.hpp"

//#include <math.h>
//#include <eigen3/Eigen/Dense>
//#include <opencv2/core/eigen.hpp>
//#include "opencv2/video/tracking.hpp"

//#include <opencv2/tracking.hpp>
//#include <opencv2/core/ocl.hpp>


//#include <opencv2/aruco.hpp>



//int main()
//{

////    cv::Mat markerImage;
////    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
////    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);


////    cv::imwrite("/home/mohit/navigation_ws/src/tracking_pkg/src/index.png", markerImage);




//    cv::Mat inputImage = cv::imread("28.png");

//    std::vector<int> markerIds;
//    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
//    cv::Ptr<cv::aruco::DetectorParameters> parameters (new(cv::aruco::DetectorParameters) );
//    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
//    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
//    std::vector<cv::Point2f> single_QR_code_4_corner_points;

//    std::cout << "marker_corners:" << std::endl;
//        for (int i=0; i<markerCorners.size(); i++)
//        {
//            single_QR_code_4_corner_points = markerCorners[i];

//            std::cout << "ID: " << markerIds[i] << ": ";
//            for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
//                std::cout << single_QR_code_4_corner_points[j] <<", ";

//            std::cout << std::endl;

//        }

//    cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);

//    std::cout << markerIds.size() << std::endl;

//   std::cout<<single_QR_code_4_corner_points[0].x<<std::endl;


//    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 515.8811205531232, 0, 321.382885003291, 0, 517.4720998172204, 265.7627959492004, 0, 0, 1);
//    cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.1724881193656085, -0.3737129326024711, 0.001483222734536619, 0.0008076929139872906, 0);

//    std::vector<cv::Vec3d> rvecs, tvecs;
//    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.02, cameraMatrix, distCoeffs, rvecs, tvecs);

//    std::cout<<tvecs[1]<<std::endl;
////    cv::aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);



//    cv::imshow("inputImage", inputImage);
//    cv::waitKey(0);


//    return 0;
//}




#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "opencv2/video/tracking.hpp"

#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


#include <opencv2/aruco.hpp>



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


typedef pcl::PointXYZRGB PointType;

geometry_msgs::PointStamped point_wrt_kinect;



int k = 0;

bool flag_img = false;
bool flag_cloud = false;
bool flag_click = false;
bool tellme=false;

int clk_row, clk_col;


using namespace cv;
namespace enc = sensor_msgs::image_encodings;
cv::Mat img;
cv::Mat current_image;


void image_call_back(const sensor_msgs::ImageConstPtr &msg)
{

  if(!flag_img)
  {
    std::cout << "\nim in imageCallback";

    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(current_image);
//    cv::imshow("image",current_image);
//    cv::waitKey(1);


    char check;

    k = 1;
    flag_img = true;
  }

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
    std::cout << "\nout in cloud_call_back";
  }

}


int main(int argc, char** argv)
{

  //    cv::Mat markerImage;
  //    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  //    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);


  //    cv::imwrite("/home/mohit/navigation_ws/src/tracking_pkg/src/index.png", markerImage);


  double white_x,white_y,white_x2,white_y2,white_x3,white_y3,white_x4,white_y4;

  ros::init(argc, argv, "test_transformation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 2, image_call_back);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 2, cloud_call_back);

  ros::AsyncSpinner spinner(4);

  spinner.start();


  while(ros::ok())
  {

    flag_img = false;
    flag_cloud = false;

    while(!flag_cloud || !flag_img || (current_point_cloud->height == 0) || (current_point_cloud->width == 0) )
    {
      usleep(10000);

    }


    cv::Mat inputImage;
    current_image.copyTo(inputImage);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters (new(cv::aruco::DetectorParameters) );
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    std::vector<cv::Point2f> single_QR_code_4_corner_points;

    std::cout << "marker_corners:" << std::endl;
    for (int i=0; i<markerCorners.size(); i++)
    {
      single_QR_code_4_corner_points = markerCorners[i];

      std::cout << "ID: " << markerIds[i] << ": ";
      for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
        std::cout << single_QR_code_4_corner_points[j] <<", ";

      std::cout << std::endl;

    }

    white_x=0; white_y=0; white_x2=0; white_y2=0; white_x3=0; white_y3=0; white_x4=0; white_y4=0;

    for (int i=0; i<markerCorners.size(); i++)
    {
      single_QR_code_4_corner_points = markerCorners[i];
       std::cout << "ID: " << markerIds[i] << ": ";

      if(markerIds[i]==16)
      {
      for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
       {
        white_x=white_x+ single_QR_code_4_corner_points[j].x;
        white_y=white_y+ single_QR_code_4_corner_points[j].y;
      }
      white_x=white_x/4;
      white_y=white_y/4;
      }

      if(markerIds[i]==17)
      {
      for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
       {
        white_x2=white_x2+ single_QR_code_4_corner_points[j].x;
        white_y2=white_y2+ single_QR_code_4_corner_points[j].y;
      }
      white_x2=white_x2/4;
      white_y2=white_y2/4;
      }

      if(markerIds[i]==18)
      {
      for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
       {
        white_x3=white_x3+ single_QR_code_4_corner_points[j].x;
        white_y3=white_y3+ single_QR_code_4_corner_points[j].y;
      }
      white_x3=white_x3/4;
      white_y3=white_y3/4;
      }

      if(markerIds[i]==19)
      {
      for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
       {
        white_x4=white_x4+ single_QR_code_4_corner_points[j].x;
        white_y4=white_y4+ single_QR_code_4_corner_points[j].y;
      }
      white_x4=white_x4/4;
      white_y4=white_y4/4;
      }
}

      std::cout<<"centroids of id: 16,17,18,19 are "<<std::endl;
      std::cout<<"("<<white_x<<","<<white_y<<"),( "<<white_x2<<","<<white_y2<<"),( "<<white_x3<<","<<white_y3<<"),( "<<white_x4<<","<<white_y4<<")"<<std::endl;


      std::cout << std::endl;



    cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);

    std::cout << markerIds.size() << std::endl;

    std::cout<<single_QR_code_4_corner_points[0].x<<std::endl;


    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 515.8811205531232, 0, 321.382885003291, 0, 517.4720998172204, 265.7627959492004, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.1724881193656085, -0.3737129326024711, 0.001483222734536619, 0.0008076929139872906, 0);


    current_image.copyTo(inputImage);

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

//    std::cout << rvecs.size() ;


     std::cout<<"markerIds "<<markerIds[0]<<" "<<tvecs[0]<<std::endl;
     std::cout<<"markerIds "<<markerIds[1]<<" "<<tvecs[1]<<std::endl;
     std::cout<<"markerIds "<<markerIds[2]<<" "<<tvecs[2]<<std::endl;
     std::cout<<"markerIds "<<markerIds[3]<<" "<<tvecs[3]<<std::endl;

     point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";
     point_wrt_kinect.point.z = current_point_cloud->at(white_x, white_y).z;
     std::cout<<"markerIds 16 "<< point_wrt_kinect.point.z<<std::endl;
     point_wrt_kinect.point.z = current_point_cloud->at(white_x2, white_y2).z;
     std::cout<<"markerIds 17 "<< point_wrt_kinect.point.z<<std::endl;
     point_wrt_kinect.point.z = current_point_cloud->at(white_x3, white_y3).z;
     std::cout<<"markerIds 18 "<< point_wrt_kinect.point.z<<std::endl;
     point_wrt_kinect.point.z = current_point_cloud->at(white_x4, white_y4).z;
     std::cout<<"markerIds 19 "<< point_wrt_kinect.point.z<<std::endl;




    for(int i=0;i<rvecs.size();i++)
    cv::aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);


//    std::cout<<"HI"<<std::endl;

    cv::imshow("inputImage", inputImage);
    cv::waitKey(1);


//    std::cout<<"HI"<<std::endl;
//    char checkme='n';
//    std::cout<<"press y to exit"<<std::endl;
//    std::cin>>checkme;
//    if(checkme=='y')
//      break;
  }
  return 0;
}


