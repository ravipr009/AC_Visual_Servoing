


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



#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>

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

#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include<moveit/robot_state/robot_state.h>



//ur5
double d1=0.0892; double d4=0.109; double d5=0.093; double d6=0.0823; double a2=-0.425; double a3=-0.39243;
//d3=0.10915 or 0

double pose[3]; double S_pose[8];

cv::Mat theta_dot = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat thetadot_prev = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat thetadouble_dot = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat theta = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat del_theta = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat rnn_w= cv::Mat(9,1, CV_64F, 0.0);
cv::Mat rnn_proj= cv::Mat(6,1, CV_64F, 0.0);


double epsi=0.1;
double gam=10;
double u_min=-0.5;
double u_max=0.5;

double st=18;
cv::Mat POSE = cv::Mat(3, 1, CV_64F, 0.0);

cv::Mat S = cv::Mat(8, 1, CV_64F, 0.0);
cv::Mat S_T = cv::Mat(8, 1, CV_64F, 0.0);
cv::Mat S_T_PREV = cv::Mat(8, 1, CV_64F, 0.0);
cv::Mat S_dot = cv::Mat(8, 1, CV_64F, 0.0);


cv::Mat err = cv::Mat(st/2, 1, CV_64F, 0.0);
cv::Mat Final_err = cv::Mat(st/2, 1, CV_64F, 0.0);


cv::Mat R = cv::Mat(6,6,CV_64F,0.0);
cv::Mat Rinv = cv::Mat(6,6,CV_64F,0.0);
cv::Mat Qbar= cv::Mat(st,st,CV_64F,0.0);
cv::Mat Q=cv::Mat(st/2,st/2,CV_64F,0.0);
cv::Mat Zeta = cv::Mat(st,1,CV_64F,0.0);
cv::Mat Zeta_T = cv::Mat(1,st,CV_64F,0.0);
cv::Mat Zeta_dot = cv::Mat(st,1,CV_64F,0.0);
cv::Mat Zeta_PREV = cv::Mat(st,1,CV_64F,0.0);

cv::Mat W1= cv::Mat(st,1,CV_64F,1.0);
cv::Mat W= cv::Mat(4,1,CV_64F,1.0);
cv::Mat W_T= cv::Mat(1,4,CV_64F,1.0);
cv::Mat W_dot = cv::Mat(4,1,CV_64F,0.0);
cv::Mat del_PHI = cv::Mat(4,st,CV_64F,0.0);
cv::Mat del_PHI_T = cv::Mat(st,4,CV_64F,0.0);
cv::Mat del_V = cv::Mat(st,1,CV_64F,0.0);
cv::Mat PHI = cv::Mat(1,4,CV_64F,0.0);
cv::Mat sig = cv::Mat(1,4  ,CV_64F,0.0);

cv::Mat re_mat(3,3,CV_64F, 0.0);
cv::Mat r2e_mat(6,6,CV_64F, 0.0);
cv::Mat te_mat(3,1,CV_64F, 0.0);
cv::Mat te_skewmat(3,3,CV_64F, 0.0);
cv::Mat tempe_mat(3,3,CV_64F, 0.0);

cv::Mat rbl_mat(3,3,CV_64F, 0.0);
cv::Mat r2bl_mat(6,6,CV_64F, 0.0);
cv::Mat tbl_mat(3,1,CV_64F, 0.0);
cv::Mat tbl_skewmat(3,3,CV_64F, 0.0);
cv::Mat tempbl_mat(3,3,CV_64F, 0.0);

cv::Mat rf_mat(6,6,CV_64F, 0.0);


cv::Mat r_mat(3,3,CV_64F, 0.0);
cv::Mat r2_mat(6,6,CV_64F, 0.0);
cv::Mat t_mat(3,1,CV_64F, 0.0);
cv::Mat t_skewmat(3,3,CV_64F, 0.0);
cv::Mat temp_mat(3,3,CV_64F, 0.0);
cv::Mat L((st/2)-1,6,CV_64F, 0.0);
cv::Mat J_new((st/2-1),6,CV_64F, 0.0);
cv::Mat J_total(st/2,6,CV_64F, 0.0);
cv::Mat J_total_pinv(6,st/2,CV_64F, 0.0);
cv::Mat J_total_T(6,st/2,CV_64F, 0.0);
cv::Mat Jp(6,6,CV_64F, 0.0);
cv::Mat Jp1(6,6,CV_64F, 0.0);

cv::Mat G = cv::Mat(st,6,CV_64F,0.0);
cv::Mat G_T = cv::Mat(6,st,CV_64F,0.0);
cv::Mat G_pinv = cv::Mat(6,st,CV_64F,0.0);

cv::Mat F = cv::Mat(st,1,CV_64F,0.0);

cv::Mat A= cv::Mat(4,4,CV_64F,1.0);
cv::Mat e_h= cv::Mat(1,1,CV_64F,1.0);
cv::Mat de_h= cv::Mat(4,1,CV_64F,1.0);
cv::Mat temp= cv::Mat(1,1,CV_64F,1.0);

cv::Mat Feature_dot= cv::Mat(st/2,1,CV_64F,0.0);


 double sd[8];double lay;
 double depth=0.5;





 std::vector<double> joints;


double alpha=0.1; double mu=1;
 int T=1;
//working
 double dt=0.01;
double t;
double sqer2;


typedef pcl::PointXYZRGB PointType;



int k = 0;

bool flag_img = false;
bool flag_cloud = false;
bool flag_click = false;
bool tellme=false;

int clk_row, clk_col;


using namespace cv;
namespace enc = sensor_msgs::image_encodings;
cv::Mat img;

unsigned int white_x=0;
unsigned int white_y=0;
unsigned int white_x2=0;
unsigned int white_y2=0;
unsigned int white_x3=0;
unsigned int white_y3=0;
unsigned int white_x4=0;
unsigned int white_y4=0;



unsigned int whitetemp_x=0;
unsigned int whitetemp_y=0;
unsigned int whitetemp_x2=0;
unsigned int whitetemp_y2=0;
unsigned int whitetemp_x3=0;
unsigned int whitetemp_y3=0;
unsigned int whitetemp_x4=0;
unsigned int whitetemp_y4=0;


using namespace std;
int thresh=15;


cv::Mat current_imageHSV;
cv::Mat current_imageBIN;
cv::Mat current_image;
cv::Mat current_imageGRY;


geometry_msgs::PointStamped point_wrt_kinect;
geometry_msgs::PointStamped point_wrt_world;




  Mat canny_output;
  vector<vector<Point> > contours;
 vector<Point> cnt;
  vector<Vec4i> hierarchy;
RNG rng(12345);
double roll = 0, pitch = 1, yaw = 3.14/180*90;

void image_call_back(const sensor_msgs::ImageConstPtr &msg)
{

  if(!flag_img)
  {
    std::cout << "\nim in imageCallback";

    current_image = cv_bridge::toCvShare(msg, "bgr8")->image ;

    k = 1;
    flag_img = true;
    usleep(10000);
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
    usleep(10000);
  }
}


int main(int argc, char** argv)
{


  ofstream myfile1;
  ofstream myfile2;
  ofstream myfile3;
  ofstream myfile4;
  ofstream myfile5;
  ofstream myfile6;
  ofstream myfile7;
  ofstream myfile8;
  ofstream myfile9;
  ofstream myfile10;
  ofstream myfile11;
  ofstream myfile12;
  ofstream myfile13;
  ofstream myfile14;
  ofstream myfile15;


  myfile1.open("error_SOTA2.txt");
  myfile2.open("cost_SOTA2.txt");
  myfile3.open("ControlVel1_SOTA2.txt");
  myfile4.open("ControlVel2_SOTA2.txt");
  myfile5.open("ControlVel3_SOTA2.txt");
  myfile6.open("ControlVel4_SOTA2.txt");
  myfile7.open("ControlVel5_SOTA2.txt");
  myfile8.open("ControlVel6_SOTA2.txt");

  myfile9.open("JointAngles1_SOTA2.txt");
  myfile10.open("JointAngles2_SOTA2.txt");
  myfile11.open("JointAngles3_SOTA2.txt");
  myfile12.open("JointAngles4_SOTA2.txt");
  myfile13.open("JointAngles5_SOTA2.txt");
  myfile14.open("JointAngles6_SOTA2.txt");

  myfile15.open("cost2_SOTA2.txt");

  ros::init(argc, argv, "test_transformation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 2, image_call_back);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 2, cloud_call_back);

  ros::AsyncSpinner spinner(4);

  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  static const std::string PLANNER ="RRTConnectkConfigDefault";



  joints = group.getCurrentJointValues();
  theta.at<double>(0,0)=joints.at(0);
  theta.at<double>(1,0)=joints.at(1);
  theta.at<double>(2,0)=joints.at(2);
  theta.at<double>(3,0)=joints.at(3);
  theta.at<double>(4,0)=joints.at(4);
  theta.at<double>(5,0)=joints.at(5);

    for(int i=0;i<st/2;i++)
      for(int j=0;j<st/2;j++)
      if(i==j)
      Q.at<double>(i,j)=1;

      for(int i=0;i<st/2;i++)
      for(int j=0;j<st/2;j++)
      Qbar.at<double>(i,j)=Q.at<double>(i,j);

      for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
        if(i==j)
        R.at<double>(i,j)=1;

      for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
        if(i==j)
        Rinv.at<double>(i,j)=1;



   geometry_msgs::PoseStamped current_Pose;
   geometry_msgs::PoseStamped current_Pose_world;


   geometry_msgs::PointStamped point_wrt_world;
   geometry_msgs::PointStamped point_wrt_kinect;

   vpServo task; // Visual servoing task
   vpFeaturePoint sd_visp;
   vpFeaturePoint s_visp;
   double u_ref=320.5;
   double v_ref=240.5;
   double px=640;
   double py=480;
   double x_visp;  //You have to compute the value of x.
   double y_visp;  //You have to compute the value of y.
   double Z_visp;  //You have to compute the value of Z.
   double xd_visp;  //You have to compute the value of x.
   double yd_visp;  //You have to compute the value of y.

   double x2_visp;  //You have to compute the value of x.
   double y2_visp;  //You have to compute the value of y.
   double xd2_visp;  //You have to compute the value of x.
   double yd2_visp;  //You have to compute the value of y.

   double x3_visp;  //You have to compute the value of x.
   double y3_visp;  //You have to compute the value of y.
   double xd3_visp;  //You have to compute the value of x.
   double yd3_visp;  //You have to compute the value of y.

   double x4_visp;  //You have to compute the value of x.
   double y4_visp;  //You have to compute the value of y.
   double xd4_visp;  //You have to compute the value of x.
   double yd4_visp;  //You have to compute the value of y.


   double u[8];
   double ud[8];

   double z,z_prev,z_d,z_dot;
   double z_d_prev=0;


   double al,be,ca,da,ea,DEL;

   double s[8];
   double sd[8];
double count=1;
bool FLAGME=false;


while(ros::ok())
{

    flag_img = false;
     flag_cloud = false;

     while(!flag_cloud | !flag_img | (current_point_cloud->height == 0) | (current_point_cloud->width == 0) )
     {
       usleep(100000);

     }
     std::cout <<"\nk: " << k;


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

     whitetemp_x=0; whitetemp_y=0; whitetemp_x2=0; whitetemp_y2=0; whitetemp_x3=0; whitetemp_y3=0; whitetemp_x4=0; whitetemp_y4=0;

     for (int i=0; i<markerCorners.size(); i++)
     {
       single_QR_code_4_corner_points = markerCorners[i];
       std::cout << "ID: " << markerIds[i] << ": ";

       if(markerIds[i]==16)
       {
         for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
         {
           whitetemp_x=whitetemp_x+ single_QR_code_4_corner_points[j].x;
           whitetemp_y=whitetemp_y+ single_QR_code_4_corner_points[j].y;
         }
         whitetemp_x=whitetemp_x/4;
         whitetemp_y=whitetemp_y/4;
       }
       if(whitetemp_x!=0 && whitetemp_y!=0)
       {
         white_x=whitetemp_x;
         white_y=whitetemp_y;
       }



       if(markerIds[i]==17)
       {
         for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
         {
           whitetemp_x2=whitetemp_x2+ single_QR_code_4_corner_points[j].x;
           whitetemp_y2=whitetemp_y2+ single_QR_code_4_corner_points[j].y;
         }
         whitetemp_x2=whitetemp_x2/4;
         whitetemp_y2=whitetemp_y2/4;
       }
       if(whitetemp_x2!=0 && whitetemp_y2!=0)
       {
         white_x2=whitetemp_x2;
         white_y2=whitetemp_y2;
       }


       if(markerIds[i]==18)
       {
         for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
         {
           whitetemp_x3=whitetemp_x3+ single_QR_code_4_corner_points[j].x;
           whitetemp_y3=whitetemp_y3+ single_QR_code_4_corner_points[j].y;
         }
         whitetemp_x3=whitetemp_x3/4;
         whitetemp_y3=whitetemp_y3/4;
       }

       if(whitetemp_x3!=0 && whitetemp_y3!=0)
       {
         white_x3=whitetemp_x3;
         white_y3=whitetemp_y3;
       }

       if(markerIds[i]==19)
       {
         for (int j=0; j<single_QR_code_4_corner_points.size(); j++)
         {
           whitetemp_x4=whitetemp_x4+ single_QR_code_4_corner_points[j].x;
           whitetemp_y4=whitetemp_y4+ single_QR_code_4_corner_points[j].y;
         }
         whitetemp_x4=whitetemp_x4/4;
         whitetemp_y4=whitetemp_y4/4;
       }

       if(whitetemp_x4!=0 && whitetemp_y4!=0)
       {
         white_x4=whitetemp_x4;
         white_y4=whitetemp_y4;
       }


     }

     std::cout<<"centroids of id: 16,17,18,19 are "<<std::endl;
     std::cout<<"("<<white_x<<","<<white_y<<"),( "<<white_x2<<","<<white_y2<<"),( "<<white_x3<<","<<white_y3<<"),( "<<white_x4<<","<<white_y4<<")"<<std::endl;


     std::cout << std::endl;
     std::cout <<"HI"<< std::endl;




     cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);

     cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 515.8811205531232, 0, 321.382885003291, 0, 517.4720998172204, 265.7627959492004, 0, 0, 1);
     cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.1724881193656085, -0.3737129326024711, 0.001483222734536619, 0.0008076929139872906, 0);
     std::cout <<"HI"<< std::endl;


     current_image.copyTo(inputImage);
     std::cout <<"HI"<< std::endl;

     std::vector<cv::Vec3d> rvecs, tvecs;
     cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

     std::cout <<"HI"<< std::endl;

     for(int i=0;i<rvecs.size();i++)
       cv::aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);


     cv::imshow("inputImage", inputImage);
     cv::waitKey(1);

  current_Pose=group.getCurrentPose();


    tf::TransformListener listener_base;
   listener_base.waitForTransform( "world", "base_link", ros::Time(0), ros::Duration(5));
   listener_base.transformPose("world",current_Pose, current_Pose_world);

   pose[0]=current_Pose_world.pose.position.x;
   pose[1]=current_Pose_world.pose.position.y;
   pose[2]=current_Pose_world.pose.position.z;

   POSE = cv::Mat(3, 1, CV_64F, pose);

    z=pose[2];
    cout<<"current z position"<<z<<endl;

    theta.at<double>(0,0)=joints.at(0);
    theta.at<double>(1,0)=joints.at(1);
    theta.at<double>(2,0)=joints.at(2);
    theta.at<double>(3,0)=joints.at(3);
    theta.at<double>(4,0)=joints.at(4);
    theta.at<double>(5,0)=joints.at(5);

//JACOBIAN CALCULATION:

                   Jp1.at<double>(0,0) = (cos(theta.at<double>(0,0))*cos(theta.at<double>(4,0))-
                               ((sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*sin(theta.at<double>(4,0)))*d6+
                               ((sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5+cos(theta.at<double>(0,0))*d4+
                               sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*a2;



                       Jp1.at<double>(0,1) = -((cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*
                               sin(theta.at<double>(4,0))*d6+((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*a2;



                       Jp1.at<double>(0,2) = -((cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*
                               sin(theta.at<double>(4,0))*d6+((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3;


                       Jp1.at<double>(0,3) = ((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               ((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0))-(cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0)))*sin(theta.at<double>(4,0))*d6;

                       Jp1.at<double>(0,4) = (-sin(theta.at<double>(0,0))*sin(theta.at<double>(4,0))-
                               ((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*cos(theta.at<double>(4,0)))*d6;

                       Jp1.at<double>(0,5) = 0.0;


                       //-----------------

                       Jp1.at<double>(1,0) = (sin(theta.at<double>(0,0))*cos(theta.at<double>(4,0))-
                               ((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*sin(theta.at<double>(4,0)))*d6+
                               ((cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5+sin(theta.at<double>(0,0))*d4-
                               cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3+cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3+cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*a2;

                       Jp1.at<double>(1,1) = -((sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*
                               sin(theta.at<double>(4,0))*d6+((-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*a2;

                       Jp1.at<double>(1,2) =  -((sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*
                               sin(theta.at<double>(4,0))*d6+((-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3;

                       Jp1.at<double>(1,3) = ((-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               ((-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0))-(sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0)))*sin(theta.at<double>(4,0))*d6;


                       Jp1.at<double>(1,4) = (cos(theta.at<double>(0,0))*sin(theta.at<double>(4,0))-
                               ((-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*cos(theta.at<double>(4,0)))*d6;

                       Jp1.at<double>(1,5) = 0;

                       //-----------------------------


                       Jp1.at<double>(2,0) = 0.0;

                       Jp1.at<double>(2,1) =  -((-cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*sin(theta.at<double>(4,0))*d6+
                               ((cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(-cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3+cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3+
                               cos(theta.at<double>(1,0))*a2;

                       Jp1.at<double>(2,2) = -((-cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*sin(theta.at<double>(4,0))*d6+
                               ((cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(-cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0))*a3+cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))*a3;

                       Jp1.at<double>(2,3) = ((cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*d5-
                               ((cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0))-(cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0)))*sin(theta.at<double>(4,0))*d6;

                       Jp1.at<double>(2,4) = -((cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*cos(theta.at<double>(4,0))*d6;

                       Jp1.at<double>(2,5) = 0.0;


                       Jp1.at<double>(3,0)= 0.0;
                       Jp1.at<double>(3,1)= sin(theta.at<double>(0,0));
                       Jp1.at<double>(3,2)= sin(theta.at<double>(0,0));

                       Jp1.at<double>(3,3)= sin(theta.at<double>(0,0));
                       Jp1.at<double>(3,4)= (cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0));

                       Jp1.at<double>(3,5)= sin(theta.at<double>(0,0))*cos(theta.at<double>(4,0))-
                               ((-cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-cos(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*sin(theta.at<double>(4,0));


                       Jp1.at<double>(4,0) = 0.0;
                       Jp1.at<double>(4,1) = -cos(theta.at<double>(0,0));
                       Jp1.at<double>(4,2) = -cos(theta.at<double>(0,0));
                       Jp1.at<double>(4,3) = -cos(theta.at<double>(0,0));

                      Jp1.at<double>(4,4)= (sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0));
                       Jp1.at<double>(4,5) = -((-sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(sin(theta.at<double>(0,0))*cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(0,0))*sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*
                               sin(theta.at<double>(4,0))-cos(theta.at<double>(0,0))*cos(theta.at<double>(4,0));
//-------------------------------------
                       Jp1.at<double>(5,0) = 1.0;
                       Jp1.at<double>(5,1)= 0.0;
                       Jp1.at<double>(5,2) = 0.0;
                       Jp1.at<double>(5,3) = 0.0;
                      Jp1.at<double>(5,4)= (cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))-(cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*cos(theta.at<double>(3,0));
                      Jp1.at<double>(5,5) = -((cos(theta.at<double>(1,0))*cos(theta.at<double>(2,0))-sin(theta.at<double>(1,0))*sin(theta.at<double>(2,0)))*sin(theta.at<double>(3,0))+(cos(theta.at<double>(1,0))*sin(theta.at<double>(2,0))+sin(theta.at<double>(1,0))*cos(theta.at<double>(2,0)))*cos(theta.at<double>(3,0)))*sin(theta.at<double>(4,0));


//cout<<"Jp wrt base: "<<Jp1<<endl;


   S_pose[0]=white_x;
   S_pose[1]=white_y;
   S_pose[2]=white_x2;
   S_pose[3]=white_y2;
   S_pose[4]=white_x3;
   S_pose[5]=white_y3;
   S_pose[6]=white_x4;
   S_pose[7]=white_y4;

   u[0]=S_pose[0];
   u[1]=S_pose[1];
   u[2]=S_pose[2];
   u[3]=S_pose[3];
   u[4]=S_pose[4];
   u[5]=S_pose[5];
   u[6]=S_pose[6];
   u[7]=S_pose[7];

   x_visp = (u[0]-u_ref)/px;
   y_visp = (u[1]-v_ref)/py;
   x2_visp = (u[2]-u_ref)/px;
   y2_visp = (u[3]-v_ref)/py;
   x3_visp = (u[4]-u_ref)/px;
   y3_visp = (u[5]-v_ref)/py;
   x4_visp = (u[6]-u_ref)/px;
   y4_visp = (u[7]-v_ref)/py;


   s[0]=x_visp;
   s[1]=y_visp;
   s[2]=x2_visp;
   s[3]=y2_visp;
   s[4]=x3_visp;
   s[5]=y3_visp;
   s[6]=x4_visp;
   s[7]=y4_visp;



       S=cv::Mat(8,1,CV_64F,s);


   point_wrt_kinect.header.frame_id = "camera_rgb_optical_frame";

   point_wrt_kinect.point.x = current_point_cloud->at(white_x, white_y).x;
    point_wrt_kinect.point.y = current_point_cloud->at(white_x, white_y).y;
    point_wrt_kinect.point.z = current_point_cloud->at(white_x, white_y).z;

 tf::TransformListener listener;
 tf::StampedTransform transform_;
 tf::StampedTransform transformbl_;


listener.waitForTransform( "world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(5));
listener.transformPoint("world", point_wrt_kinect, point_wrt_world);

listener.lookupTransform("camera_rgb_optical_frame", "base",  ros::Time(0), transform_);
listener.lookupTransform("base", "base",  ros::Time(0), transformbl_);


depth=point_wrt_kinect.point.z;



ud[0]=424;
ud[1]=115;

ud[2]=265;
ud[3]=103;

ud[4]=253;
ud[5]=338;

ud[6]=417;
ud[7]=345;

z_d=0.53;



//ud[0]=426;
//ud[1]=139;

//ud[2]=229;
//ud[3]=119;

//ud[4]=202;
//ud[5]=413;

//ud[6]=409;
//ud[7]=428;

//z_d=0.416;


//ud[0]=357;
//ud[1]=247;

//ud[2]=311;
//ud[3]=208;

//ud[4]=277;
//ud[5]=247;

//ud[6]=312;
//ud[7]=285;

//z_d=0.28;



   xd_visp = (ud[0]-u_ref)/px;
    yd_visp = (ud[1]-v_ref)/py;
    xd2_visp = (ud[2]-u_ref)/px;
     yd2_visp = (ud[3]-v_ref)/py;
    xd3_visp = (ud[4]-u_ref)/px;
     yd3_visp = (ud[5]-v_ref)/py;
    xd4_visp = (ud[6]-u_ref)/px;
     yd4_visp = (ud[7]-v_ref)/py;

    Z_visp=depth;
    sd[0]=xd_visp;
    sd[1]=yd_visp;
    sd[2]=xd2_visp;
    sd[3]=yd2_visp;
    sd[4]=xd3_visp;
    sd[5]=yd3_visp;
    sd[6]=xd4_visp;
    sd[7]=yd4_visp;


    S_T = cv::Mat(8, 1, CV_64F, sd);
     err.at<double>(0,0) =  S.at<double>(0,0)-S_T.at<double>(0,0);
       err.at<double>(1,0) =  S.at<double>(1,0)-S_T.at<double>(1,0);
       err.at<double>(2,0) =  S.at<double>(2,0)-S_T.at<double>(2,0);
         err.at<double>(3,0) =  S.at<double>(3,0)-S_T.at<double>(3,0);
     err.at<double>(4,0) =  S.at<double>(4,0)-S_T.at<double>(4,0);
       err.at<double>(5,0) =  S.at<double>(5,0)-S_T.at<double>(5,0);
       err.at<double>(6,0) =  S.at<double>(6,0)-S_T.at<double>(6,0);
         err.at<double>(7,0) =  S.at<double>(7,0)-S_T.at<double>(7,0);
       err.at<double>(8,0)=z-z_d;

       Zeta.at<double>(0,0)= err.at<double>(0,0);
       Zeta.at<double>(1,0)= err.at<double>(1,0);
      Zeta.at<double>(2,0)=err.at<double>(2,0);
      Zeta.at<double>(3,0)=err.at<double>(3,0);
     Zeta.at<double>(4,0)=err.at<double>(4,0);
       Zeta.at<double>(5,0)=err.at<double>(5,0);
       Zeta.at<double>(6,0)=err.at<double>(6,0);
      Zeta.at<double>(7,0)=err.at<double>(7,0);
      Zeta.at<double>(8,0)=err.at<double>(8,0);


      Zeta.at<double>(9,0)= sd[0];
      Zeta.at<double>(10,0)= sd[1];
     Zeta.at<double>(11,0)= sd[2];
       Zeta.at<double>(12,0)= sd[3];
      Zeta.at<double>(13,0)= sd[4];
      Zeta.at<double>(14,0)= sd[5];
     Zeta.at<double>(15,0)= sd[6];
       Zeta.at<double>(16,0)= sd[7];
      Zeta.at<double>(17,0)= z_d;


//(2) best out of 3: dt=0.01 (this was the one which was used last)
        double norme_z;
        sig.at<double>(0,0)=1; sig.at<double>(0,1)=0.75; sig.at<double>(0,2)=0.5; sig.at<double>(0,3)=0.0000000001;

        norme_z=Zeta.at<double>(0,0)*Zeta.at<double>(0,0)+Zeta.at<double>(1,0)*Zeta.at<double>(1,0)+Zeta.at<double>(2,0)*Zeta.at<double>(2,0)+Zeta.at<double>(3,0)*Zeta.at<double>(3,0)+Zeta.at<double>(4,0)*Zeta.at<double>(4,0)+Zeta.at<double>(5,0)*Zeta.at<double>(5,0)+Zeta.at<double>(6,0)*Zeta.at<double>(6,0)+Zeta.at<double>(7,0)*Zeta.at<double>(7,0)+Zeta.at<double>(8,0)*Zeta.at<double>(8,0);

        for(int i=0;i<4;i++)
          PHI.at<double>(0,i)=exp(-(norme_z)/(2*sig.at<double>(0,i)*sig.at<double>(0,i)));

        cout<<"phi"<<PHI<<endl;


           for(int i=0;i<4;i++)
             for(int j=0;j<st/2;j++)
               del_PHI.at<double>(i,j)= PHI.at<double>(0,i)*(0.5/(sig.at<double>(0,i)*sig.at<double>(0,i)))*2*Zeta.at<double>(j,0);


cout<<"del_phi"<<del_PHI<<endl;

             cv::transpose(del_PHI,del_PHI_T);
             del_V=del_PHI_T*W;

                 L.at<double>(0,0)=-1/(Z_visp);
                 L.at<double>(0,1)=0;
                 L.at<double>(0,2)=x_visp/(Z_visp);
                 L.at<double>(0,3)=x_visp*y_visp;
                 L.at<double>(0,4)=-(1+x_visp*x_visp);
                 L.at<double>(0,5)=y_visp;

                L.at<double>(1,0)=0;
                  L.at<double>(1,1)=-1/(Z_visp);
                  L.at<double>(1,2)=(y_visp)/(Z_visp);
                  L.at<double>(1,3)=(1+y_visp*y_visp);
                    L.at<double>(1,4)=-x_visp*y_visp;
                    L.at<double>(1,5)=-x_visp;

                    L.at<double>(2,0)=-1/(Z_visp);
                    L.at<double>(2,1)=0;
                    L.at<double>(2,2)=x2_visp/(Z_visp);
                    L.at<double>(2,3)=x2_visp*y2_visp;
                    L.at<double>(2,4)=-(1+x2_visp*x2_visp);
                    L.at<double>(2,5)=y2_visp;

                   L.at<double>(3,0)=0;
                     L.at<double>(3,1)=-1/(Z_visp);
                     L.at<double>(3,2)=(y2_visp)/(Z_visp);
                     L.at<double>(3,3)=(1+y2_visp*y2_visp);
                       L.at<double>(3,4)=-x2_visp*y2_visp;
                       L.at<double>(3,5)=-x2_visp;

                 L.at<double>(4,0)=-1/(Z_visp);
                 L.at<double>(4,1)=0;
                 L.at<double>(4,2)=x3_visp/(Z_visp);
                 L.at<double>(4,3)=x3_visp*y3_visp;
                 L.at<double>(4,4)=-(1+x3_visp*x3_visp);
                 L.at<double>(4,5)=y3_visp;

                L.at<double>(5,0)=0;
                  L.at<double>(5,1)=-1/(Z_visp);
                  L.at<double>(5,2)=(y3_visp)/(Z_visp);
                  L.at<double>(5,3)=(1+y3_visp*y3_visp);
                    L.at<double>(5,4)=-x3_visp*y3_visp;
                    L.at<double>(5,5)=-x3_visp;

                    L.at<double>(6,0)=-1/(Z_visp);
                    L.at<double>(6,1)=0;
                    L.at<double>(6,2)=x4_visp/(Z_visp);
                    L.at<double>(6,3)=x4_visp*y4_visp;
                    L.at<double>(6,4)=-(1+x4_visp*x4_visp);
                    L.at<double>(6,5)=y4_visp;

                   L.at<double>(7,0)=0;
                     L.at<double>(7,1)=-1/(Z_visp);
                     L.at<double>(7,2)=(y4_visp)/(Z_visp);
                     L.at<double>(7,3)=(1+y4_visp*y4_visp);
                       L.at<double>(7,4)=-x4_visp*y4_visp;
                       L.at<double>(7,5)=-x4_visp;


                  transform_.getBasis().getRPY(roll,pitch,yaw);



                   tf::Transform tf;
                   tf::Vector3 tfVec;
                    tf::Quaternion quat;
                   tf::Matrix3x3 tfR;

                   tfVec=transform_.getOrigin();
                  //  cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
                   t_mat.at<double>(0,0)=tfVec.getX(); t_mat.at<double>(1,0)=tfVec.getY(); t_mat.at<double>(2,0)=tfVec.getZ();
                   t_skewmat.at<double>(0,1)=-t_mat.at<double>(2,0);
                   t_skewmat.at<double>(0,2)=t_mat.at<double>(1,0);
                   t_skewmat.at<double>(1,0)=t_mat.at<double>(2,0);
                   t_skewmat.at<double>(1,2)=-t_mat.at<double>(0,0);
                   t_skewmat.at<double>(2,0)=-t_mat.at<double>(1,0);
                   t_skewmat.at<double>(2,1)=t_mat.at<double>(0,0);

                    tfR=transform_.getBasis();

                    tfVec = tfR.getRow(0);
                        r_mat.at<double>(0,0)=tfVec.getX();
                        r_mat.at<double>(0,1)=tfVec.getY();
                        r_mat.at<double>(0,2)=tfVec.getZ();

                        tfVec = tfR.getRow(1);
                        r_mat.at<double>(1,0)=tfVec.getX();
                        r_mat.at<double>(1,1)=tfVec.getY();
                        r_mat.at<double>(1,2)=tfVec.getZ();

                        tfVec = tfR.getRow(2);
                        r_mat.at<double>(2,0)=tfVec.getX();
                        r_mat.at<double>(2,1)=tfVec.getY();
                        r_mat.at<double>(2,2)=tfVec.getZ();

                        temp_mat=t_skewmat*r_mat;

                    for(int i=0;i<3;i++)
                      for(int j=0;j<3;j++)
                        r2_mat.at<double>(i,j)=r_mat.at<double>(i,j);


                   for(int i=0;i<3;i++)
                          for(int j=3;j<6;j++)
                                 r2_mat.at<double>(i,j)=temp_mat.at<double>(i,j-3);

                        for(int i=3;i<6;i++)
                          for(int j=3;j<6;j++)
                            r2_mat.at<double>(i,j)=r_mat.at<double>(i-3,j-3);



//--------------------------------------------------------------------------------------------------
                              transformbl_.getBasis().getRPY(roll,pitch,yaw);



                               tfVec=transformbl_.getOrigin();
                              //  cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
                               tbl_mat.at<double>(0,0)=tfVec.getX(); tbl_mat.at<double>(1,0)=tfVec.getY(); tbl_mat.at<double>(2,0)=tfVec.getZ();
                               tbl_skewmat.at<double>(0,1)=-tbl_mat.at<double>(2,0);
                               tbl_skewmat.at<double>(0,2)=tbl_mat.at<double>(1,0);
                               tbl_skewmat.at<double>(1,0)=tbl_mat.at<double>(2,0);
                               tbl_skewmat.at<double>(1,2)=-tbl_mat.at<double>(0,0);
                               tbl_skewmat.at<double>(2,0)=-tbl_mat.at<double>(1,0);
                               tbl_skewmat.at<double>(2,1)=tbl_mat.at<double>(0,0);


                                tfR=transformbl_.getBasis();

                                tfVec = tfR.getRow(0);
                                    rbl_mat.at<double>(0,0)=tfVec.getX();
                                    rbl_mat.at<double>(0,1)=tfVec.getY();
                                    rbl_mat.at<double>(0,2)=tfVec.getZ();

                                    tfVec = tfR.getRow(1);
                                    rbl_mat.at<double>(1,0)=tfVec.getX();
                                    rbl_mat.at<double>(1,1)=tfVec.getY();
                                    rbl_mat.at<double>(1,2)=tfVec.getZ();

                                    tfVec = tfR.getRow(2);
                                    rbl_mat.at<double>(2,0)=tfVec.getX();
                                    rbl_mat.at<double>(2,1)=tfVec.getY();
                                    rbl_mat.at<double>(2,2)=tfVec.getZ();

                                    tempbl_mat=te_skewmat*rbl_mat;

                                for(int i=0;i<3;i++)
                                  for(int j=0;j<3;j++)
                                    r2bl_mat.at<double>(i,j)=rbl_mat.at<double>(i,j);


                               for(int i=0;i<3;i++)
                                      for(int j=3;j<6;j++)
                                             r2bl_mat.at<double>(i,j)=0;

                                    for(int i=3;i<6;i++)
                                      for(int j=3;j<6;j++)
                                        r2bl_mat.at<double>(i,j)=rbl_mat.at<double>(i-3,j-3);


                                    Jp=r2bl_mat*Jp1;



                    J_new=L*r2_mat*Jp;


                    for(int i=0;i<(st/2)-1;i++)
                                          for(int j=0;j<6;j++)
                                            J_total.at<double>(i,j)=J_new.at<double>(i,j);

                                        int h=(st/2)-1;
                                        for(int j=0;j<6;j++)
                                          J_total.at<double>(h,j)=Jp.at<double>(2,j);

                                        cv::invert(J_total, J_total_pinv, DECOMP_SVD);
                                        std::cout<<"PINV of J_TOTAL: "<<J_total_pinv<<std::endl;

             for (int i=0;i<st/2;i++)
                 for (int j=0;j<6;j++)
                     G.at<double>(i,j)=J_total.at<double>(i,j);


             cv::transpose(G,G_T);
             cv::transpose(Zeta,Zeta_T);
             cv::transpose(W,W_T);


             S_dot=(S_T-S_T_PREV)/dt;

             z_dot=(z_d-z_d_prev)/dt;

               F.at<double>(0,0)=-S_dot.at<double>(0,0);
               F.at<double>(1,0)=-S_dot.at<double>(1,0);
               F.at<double>(2,0)=-S_dot.at<double>(2,0);
               F.at<double>(3,0)=-S_dot.at<double>(3,0);
               F.at<double>(4,0)=-S_dot.at<double>(4,0);
               F.at<double>(5,0)=-S_dot.at<double>(5,0);
               F.at<double>(6,0)=-S_dot.at<double>(6,0);
               F.at<double>(7,0)=-S_dot.at<double>(7,0);
               F.at<double>(8,0)=-z_dot;

               F.at<double>(9,0)=S_dot.at<double>(0,0);
               F.at<double>(10,0)=S_dot.at<double>(1,0);
               F.at<double>(11,0)=S_dot.at<double>(2,0);
               F.at<double>(12,0)=S_dot.at<double>(3,0);
               F.at<double>(13,0)=S_dot.at<double>(4,0);
               F.at<double>(14,0)=S_dot.at<double>(5,0);
               F.at<double>(15,0)=S_dot.at<double>(6,0);
               F.at<double>(16,0)=S_dot.at<double>(7,0);
               F.at<double>(17,0)=z_dot;


Zeta_dot=(Zeta-Zeta_PREV)/dt;



Zeta_PREV.at<double>(0,0)=Zeta.at<double>(0,0);Zeta_PREV.at<double>(1,0)=Zeta.at<double>(1,0);Zeta_PREV.at<double>(2,0)=Zeta.at<double>(2,0);
Zeta_PREV.at<double>(3,0)=Zeta.at<double>(3,0);Zeta_PREV.at<double>(4,0)=Zeta.at<double>(4,0);Zeta_PREV.at<double>(5,0)=Zeta.at<double>(5,0);
Zeta_PREV.at<double>(6,0)=Zeta.at<double>(6,0);Zeta_PREV.at<double>(7,0)=Zeta.at<double>(7,0);Zeta_PREV.at<double>(8,0)=Zeta.at<double>(8,0);
Zeta_PREV.at<double>(9,0)=Zeta.at<double>(9,0);Zeta_PREV.at<double>(10,0)=Zeta.at<double>(10,0);Zeta_PREV.at<double>(11,0)=Zeta.at<double>(11,0);
Zeta_PREV.at<double>(12,0)=Zeta.at<double>(12,0);Zeta_PREV.at<double>(13,0)=Zeta.at<double>(13,0);Zeta_PREV.at<double>(14,0)=Zeta.at<double>(14,0);
Zeta_PREV.at<double>(15,0)=Zeta.at<double>(15,0);Zeta_PREV.at<double>(16,0)=Zeta.at<double>(16,0);Zeta_PREV.at<double>(17,0)=Zeta.at<double>(17,0);


rnn_w=rnn_w-(dt/epsi)*gam*err-(dt/epsi)*J_total*theta_dot;
 cv::transpose(J_total,J_total_T);
 rnn_proj=J_total_T*rnn_w;

for(int i=0;i<6;i++)
 { if(rnn_proj.at<double>(i,0)< (-0.5))
    rnn_proj.at<double>(i,0)=-0.5;

    if(rnn_proj.at<double>(i,0)>0.5)
      rnn_proj.at<double>(i,0)=0.5;
}

thetadot_prev=theta_dot;
theta_dot=rnn_proj;
thetadouble_dot=(theta_dot-thetadot_prev)/dt;

           sqer2=(Zeta.at<double>(2,0)* Zeta.at<double>(2,0)+Zeta.at<double>(0,0)* Zeta.at<double>(0,0)+Zeta.at<double>(1,0)* Zeta.at<double>(1,0)+Zeta.at<double>(3,0)* Zeta.at<double>(3,0)+Zeta.at<double>(4,0)* Zeta.at<double>(4,0)+Zeta.at<double>(5,0)* Zeta.at<double>(5,0)+Zeta.at<double>(6,0)* Zeta.at<double>(6,0)+Zeta.at<double>(7,0)* Zeta.at<double>(7,0))/8;

              Zeta=Zeta+dt*(F+G*theta_dot);

              theta.at<double>(0,0)=theta.at<double>(0,0) + dt*theta_dot.at<double>(0,0);
              theta.at<double>(1,0)=theta.at<double>(1,0) + dt*theta_dot.at<double>(1,0);
              theta.at<double>(2,0)=theta.at<double>(2,0) + dt*theta_dot.at<double>(2,0);
              theta.at<double>(3,0)=theta.at<double>(3,0) + dt*theta_dot.at<double>(3,0);
              theta.at<double>(4,0)=theta.at<double>(4,0) + dt*theta_dot.at<double>(4,0);
              theta.at<double>(5,0)=theta.at<double>(5,0) + dt*theta_dot.at<double>(5,0);
              cout<<"THETA for wrt base_link: "<<theta<<endl;



              joints.resize(6);

              joints.at(0)=theta.at<double>(0,0);
              joints.at(1)=theta.at<double>(1,0);
              joints.at(2)=theta.at<double>(2,0);
              joints.at(3)=theta.at<double>(3,0);
              joints.at(4)=theta.at<double>(4,0);
              joints.at(5)=theta.at<double>(5,0);



              myfile3<<theta_dot.at<double>(0,0)<<endl;
              myfile4<<theta_dot.at<double>(1,0)<<endl;
              myfile5<<theta_dot.at<double>(2,0)<<endl;
              myfile6<<theta_dot.at<double>(3,0)<<endl;
              myfile7<<theta_dot.at<double>(4,0)<<endl;
              myfile8<<theta_dot.at<double>(5,0)<<endl;

              myfile9<<theta.at<double>(0,0)<<endl;
              myfile10<<theta.at<double>(1,0)<<endl;
              myfile11<<theta.at<double>(2,0)<<endl;
              myfile12<<theta.at<double>(3,0)<<endl;
              myfile13<<theta.at<double>(4,0)<<endl;
              myfile14<<theta.at<double>(5,0)<<endl;


              group.setJointValueTarget(joints);
              group.move();
              usleep(10000);

              joints = group.getCurrentJointValues();
              cout<<"joint angles "<<joints[1]<<joints[2]<<joints[3]<<endl;


              double cost=0;
              for(int i=0;i<6;i++)
                cost=cost+theta_dot.at<double>(i,0)*theta_dot.at<double>(i,0)+Zeta.at<double>(i,0)*Zeta.at<double>(i,0);



              double cost2=0;
              for(int i=0;i<st/2;i++)
                cost2=cost2+Zeta.at<double>(i,0)*Zeta.at<double>(i,0)+theta_dot.at<double>(i,0)*theta_dot.at<double>(i,0)+thetadouble_dot.at<double>(i,0)*thetadouble_dot.at<double>(i,0);

              cout<<"COST: "<<cost<<endl;
              std::cout<<"squared error "<<sqer2<<std::endl;

              myfile1<<sqer2<<endl;
              myfile2<<cost<<endl;
               myfile15<<cost2<<endl;
//              geometry_msgs::PoseStamped current_Pose;
              geometry_msgs::PoseStamped current_Pose_base;



          z_d_prev=z_d;

            for(int ij = 0; ij < 8; ij++)
            S_T_PREV.at<double>(ij,0)=sd[ij];


            if(t==T)
              break;
if(sqer2<=0.0003)
  break;
  t=t+dt;

  }

   myfile1.close();
   myfile2.close();
   myfile3.close();
   myfile4.close();
   myfile5.close();
   myfile6.close();
   myfile7.close();
   myfile8.close();
   myfile9.close();
   myfile10.close();
   myfile11.close();
   myfile12.close();
   myfile13.close();
   myfile14.close();
   myfile15.close();


  std::cout<<"squared error2 "<<sqer2<<std::endl;
  return 0;
    }

