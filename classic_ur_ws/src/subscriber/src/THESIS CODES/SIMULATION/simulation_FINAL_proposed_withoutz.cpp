
////----------------------------------------------------------------------------------------------------HSV---------------------------------------------------------------------------
////-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#include <iostream>
//#include "opencv2/opencv.hpp"

//using namespace std;
//using namespace cv;

//#define CAMERA0 0


//float data1[2][2];
//float data2[2][2];
//unsigned int Rclick, Lclick;

//void HSVTuning()

//{
//  cout<<"HI 9"<<endl;
//        cout<<"HSV Tunning of Camera "<<endl;
//        Mat frame, frameHSV, frameBIN;
//        char key;
//        int filter[6];
//        cout<<"HI 10"<<endl;



//        namedWindow("Parameters"); // Windoe to display Parameter trackbar
//        createTrackbar( "Hmax", "Parameters", &filter[0], 255, NULL);
//        createTrackbar( "Hmin", "Parameters", &filter[1], 255, NULL);
//        createTrackbar( "Smax", "Parameters", &filter[2], 255, NULL);
//        createTrackbar( "Smin", "Parameters", &filter[3], 255, NULL);
//        createTrackbar( "Vmax", "Parameters", &filter[4], 255, NULL);
//        createTrackbar( "Vmin", "Parameters", &filter[5], 255, NULL);

//        cout<<"HI 11"<<endl;
//        for(;;)
//        {
//                frame=imread("view3.jpg",IMREAD_COLOR);

//                key = waitKey(5);
//                cvtColor(frame, frameHSV, CV_BGR2HSV); // Convert colour to HSV

//                inRange(frameHSV, Scalar(filter[1], filter[3], filter[5]), Scalar(filter[0], filter[2], filter[4]), frameBIN); //Filter HSV image to bet Buinary image

//                if (key == 'q')
//                        break;
//                imshow("Camera", frame);
//                imshow("Camera HSV", frameHSV);
//                imshow("Camera BIN", frameBIN);
//        }
//        cout<<"HI 12"<<endl;
//        destroyAllWindows();
//}

////void OrientationOfCamera()
////{
////	cout<<"Check Orientation of Camera "<<endl;
////	Mat frame;
////	char key;
////	namedWindow("Camera 0"); // Window to display input RGB image

////	for(;;)
////	{
////		frame=imread("view.jpg",IMREAD_COLOR);
////		key = waitKey(5);
////		line(frame, Point(320, 0), Point(320, 480), Scalar(0,0,255), 1, 8);
////		if (key == 'q')
////			break;
////		imshow("Camera 0", frame);
////	}
////	destroyAllWindows();
////}

//void CallBackFunc(int event, int x, int y, int flags, void* userdata)
//{

//  cout<<"HI 8"<<endl;
//        if  ( event == EVENT_RBUTTONDOWN )
//        {
//                cout << "mouse is R clicked - position (" << x << ", " << y << ")" << endl;
//                data1[Rclick][0] = x;
//                data1[Rclick][1] = y;
//                Rclick++;
//        }

//        if  ( event == EVENT_LBUTTONDOWN )
//        {
//                cout << "mouse is L clicked - position (" << x << ", " << y << ")" << endl;
//                data2[Lclick][0] = x;
//                data2[Lclick][1] = y;
//                Lclick++;
//        }
//}

//bool Intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
//{

//  cout<<"HI 7"<<endl;
//    Point2f x = o2 - o1;
//    Point2f d1 = p1 - o1;
//    Point2f d2 = p2 - o2;

//    float cross = d1.x*d2.y - d1.y*d2.x;
//    if (abs(cross) < 1e-8)
//        return false;

//    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
//    r = o1 + d1 * t1;
//    return true;
//}



//void OriginOfThread()
//{
//        unsigned short int i;
//        Mat frame;
//        Rclick = Lclick = 0;
//        char key = 'r';
//        namedWindow("Camera"); // Window to display input RGB image

//        cout<<"HI 6"<<endl;

//        for(i=0; i<750; i++)
//        {

//                cout<<i<<endl;
//                key = waitKey(10);
//                if (key == 'q')
//                        break;
//                imshow("Camera", frame);

//        }
//        cout<<endl;



//        Point2f r;


//        setMouseCallback("Camera", CallBackFunc, NULL);

//        cout<<"HI 5"<<endl;
//        for(;;)
//        {

//                imshow("Camera",frame);
//                key = waitKey(0);
//                if (key == 'q')
//                        break;


//                else if(key == 'c')
//                {
//                        Intersection(Point2f(data1[0][0],data1[0][1]), Point2f(data1[1][0],data1[1][1]), Point2f(data2[0][0],data2[0][1]), Point2f(data2[1][0],data2[1][1]), r);
//                        cout<<r<<endl;
//                        Rclick = 0;
//                        Lclick = 0;
//                }
//        }
//}

//void LengthOfThread()
//{

//  cout<<"HI 3"<<endl;
//        Mat frame;

//        float origin[2];
//        cout<<" Enter center ";
//        cin>>origin[0]>>origin[1];

//        setMouseCallback("Camera", CallBackFunc, NULL);
//        char key = 'r';

//        cout<<"HI 4"<<endl;
//        for(;;)
//        {
//                Rclick = 0;

//                imshow("Camera",frame);
//                key = waitKey(0);
//                if (key == 'q')
//                        break;
//                else if(key == 'n')
//                {
//                        cout<<sqrt((data1[0][0] - origin[0])*(data1[0][0] - origin[0]) + (data1[0][1] - origin[1])*(data1[0][1] - origin[1]))<<endl;
//                }
//        }
//}

//int main(int argc, char** argv)
//{



//        // OpenCV -------------------------

//        Mat Frame[2];
//        char key;


//        // C++ -------------------------
//        unsigned int i;
//        cout<<"HI 1"<<endl;


////	cout<<" ===Orientation Caliberation==="<<endl;
////	for(i=0; i<2; i++)
//        //{
//        //	OrientationOfCamera(i, Camera[i]);
//        //	cout<<" Orientation of camera "<<i<<" Done!!!"<<endl;
//        //}

////	cout<<" ===Length Caliberation"<<endl;
////	cout<<" Which camera to use for origin calculation? ";
////	cin>>i;
////	OriginOfThread(Camera[i]);

////	cout<<" ===Distance Caliberation"<<endl;
////	LengthOfThread(Camera[i]);

//        cout<<"HI PEOPLE ===HSV Caliberation==="<<endl;
//        for(i=0; i<1; i++)
//        {
//                HSVTuning();
//                cout<<" HSV Tunning of camera Done!!!"<<endl;
//        }


//        cout<<"HI 2"<<endl;



//        return 0;
//}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


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
cv::Mat theta = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat theta1_dot = cv::Mat(6,1, CV_64F, 0.0);
cv::Mat theta1 = cv::Mat(6,1, CV_64F, 0.0);

double st=16;
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
cv::Mat L((st/2),6,CV_64F, 0.0);
cv::Mat J_new((st/2),6,CV_64F, 0.0);
cv::Mat J_total(st/2,6,CV_64F, 0.0);
cv::Mat Jp(6,6,CV_64F, 0.0);
cv::Mat Jp1(6,6,CV_64F, 0.0);

cv::Mat G = cv::Mat(st,6,CV_64F,0.0);
cv::Mat G_T = cv::Mat(6,st,CV_64F,0.0);

cv::Mat F = cv::Mat(st,1,CV_64F,0.0);

cv::Mat A= cv::Mat(4,4,CV_64F,1.0);
cv::Mat e_h= cv::Mat(1,1,CV_64F,1.0);
cv::Mat de_h= cv::Mat(4,1,CV_64F,1.0);
cv::Mat temp= cv::Mat(1,1,CV_64F,1.0);




 double sd[8];double lay;
 double depth=0.5;





 std::vector<double> joints;


double lam=1; double alpha=0.1; double mu=1;
 int T=1;
//working
 double dt=0.018;
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



using namespace std;
int thresh=15;


cv::Mat current_imageHSV;
cv::Mat current_imageBIN;
cv::Mat current_image;
cv::Mat current_imageGRY;


geometry_msgs::PointStamped point_wrt_kinect;
geometry_msgs::PointStamped point_wrt_world;





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
  ofstream myfile16;
  ofstream myfile17;
  ofstream myfile18;

  myfile1.open("error_MYCODE.txt");
  myfile2.open("cost_MYCODE.txt");
  myfile3.open("ControlVel1_MYCODE.txt");
  myfile4.open("ControlVel2_MYCODE.txt");
  myfile5.open("ControlVel3_MYCODE.txt");
  myfile6.open("ControlVel4_MYCODE.txt");
  myfile7.open("ControlVel5_MYCODE.txt");
  myfile8.open("ControlVel6_MYCODE.txt");

  myfile9.open("JointAngles1_MYCODE.txt");
  myfile10.open("JointAngles2_MYCODE.txt");
  myfile11.open("JointAngles3_MYCODE.txt");
  myfile12.open("JointAngles4_MYCODE.txt");
  myfile13.open("JointAngles5_MYCODE.txt");
  myfile14.open("JointAngles6_MYCODE.txt");

  myfile15.open("Weight1.txt");
  myfile16.open("Weight2.txt");
  myfile17.open("Weight3.txt");
  myfile18.open("Weight4.txt");

  ros::init(argc, argv, "test_transformation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image> ("camera1/image_raw", 2, image_call_back);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("/camera1/depth/points", 1, cloud_call_back);

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
   double z,z_d,z_dot;
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


      cv::waitKey(30);
        cv::Mat result(current_image.rows,current_image.cols, CV_8UC3, cv::Scalar(0,0,0));


          char key2;
          unsigned int i,j=0;
          unsigned int t = 0;

//(1) red

                  unsigned int white_count =  0; ////Count of all white pixels

                  vector <unsigned int> centroid_x;
                  vector <unsigned int> centroid_y;

                          white_x = 0;
                          white_y = 0;
                          white_count = 0;


                         int filter[6]={0,0,255,25,255,0};




                          cvtColor(current_image, current_imageHSV, CV_BGR2HSV);
                          cvtColor(current_image, current_imageGRY, CV_BGR2HSV);// Convert colour to HSV

                        cv::inRange(current_imageHSV, cv::Scalar(filter[1], filter[3], filter[5]), cv::Scalar(filter[0], filter[2], filter[4]),current_imageBIN); //Filter HSV image to bet Buinary image

                          imshow("Camera", current_image);


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



                                         circle(current_image, Point(white_x, white_y), 5, Scalar(0,0,0), 2, 8, 0);

                                           cout<<"("<<white_x<<","<<white_y<<")"<<endl;

                                         imshow("Camera", current_image);




                                                                                //(2)green

                                                                                      double white_x2 = 0;
                                                                                      double white_y2 = 0;
                                                                                      unsigned int white_count2 = 0;
                                                                                      centroid_x.clear();
                                                                                      centroid_y.clear();

                                                                                        int filter2[6]={105,33,255,0,255,0};



                                                                                      cvtColor(current_image, current_imageHSV, CV_BGR2HSV);
                                                                                      cvtColor(current_image, current_imageGRY, CV_BGR2HSV);// Convert colour to HSV

                                                                                    cv::inRange(current_imageHSV, cv::Scalar(filter2[1], filter2[3], filter2[5]), cv::Scalar(filter2[0], filter2[2], filter2[4]),current_imageBIN); //Filter HSV image to bet Buinary image

                                                                                      imshow("Camera", current_image);


                                                                                      for(unsigned int i=0; i<current_imageBIN.rows; i++)
                                                                                                     {
                                                                                                             for(unsigned int j=0; j<current_imageBIN.cols; j++)
                                                                                                             {
                                                                                                                     if(current_imageBIN.data[i*current_imageBIN.cols + j] > 200) // If colour is white
                                                                                                                     {
                                                                                                                             white_count2++; // Number of white pixels
                                                                                                                             white_x2 += j; //
                                                                                                                             white_y2 += i; //
                                                                                                                     }
                                                                                                             }
                                                                                                     }

                                                                                                     if(white_count2 > thresh)
                                                                                                     {
                                                                                                             white_x2 /= white_count2;
                                                                                                             white_y2 /= white_count2;
                                                                                                             centroid_x.push_back(white_x2);
                                                                                                             centroid_y.push_back(white_y2);
                                                                                                     }
                                                                                                     else
                                                                                                     {
                                                                                                             white_x2 = 0;
                                                                                                             white_y2 = 0;
                                                                                                             centroid_x.clear();
                                                                                                             centroid_y.clear();
                                                                                                           result = Scalar(0,0,0);
                                                                                                     }



                                                                                                     if(centroid_x.size() > 1)
                                                                                                     {

                                                                                                     line(result, Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Scalar(255,255,255), 5,8);


                                                                                                     }



                                                                                                     circle(current_image, Point(white_x2, white_y2), 5, Scalar(255,255,0), 2, 8, 0);

                                                                                                       cout<<"("<<white_x2<<","<<white_y2<<")"<<endl;


//(3) blue

                                                                                      double white_x3 = 0;
                                                                                      double white_y3 = 0;
                                                                                      unsigned int white_count3 = 0;
                                                                                      centroid_x.clear();
                                                                                      centroid_y.clear();


                                                                                       int filter3[6]={255,115,255,0,255,0};



                                                                                      cvtColor(current_image, current_imageHSV, CV_BGR2HSV);
                                                                                      cvtColor(current_image, current_imageGRY, CV_BGR2HSV);// Convert colour to HSV

                                                                                    cv::inRange(current_imageHSV, cv::Scalar(filter3[1], filter3[3], filter3[5]), cv::Scalar(filter3[0], filter3[2], filter3[4]),current_imageBIN); //Filter HSV image to bet Buinary image

                                                                                      imshow("Camera", current_image);


                                                                                      for(unsigned int i=0; i<current_imageBIN.rows; i++)
                                                                                                     {
                                                                                                             for(unsigned int j=0; j<current_imageBIN.cols; j++)
                                                                                                             {
                                                                                                                     if(current_imageBIN.data[i*current_imageBIN.cols + j] > 200) // If colour is white
                                                                                                                     {
                                                                                                                             white_count3++; // Number of white pixels
                                                                                                                             white_x3 += j; //
                                                                                                                             white_y3 += i; //
                                                                                                                     }
                                                                                                             }
                                                                                                     }

                                                                                                     if(white_count3 > thresh)
                                                                                                     {
                                                                                                             white_x3 /= white_count3;
                                                                                                             white_y3 /= white_count3;
                                                                                                             centroid_x.push_back(white_x3);
                                                                                                             centroid_y.push_back(white_y3);
                                                                                                     }
                                                                                                     else
                                                                                                     {
                                                                                                             white_x3 = 0;
                                                                                                             white_y3 = 0;
                                                                                                             centroid_x.clear();
                                                                                                             centroid_y.clear();
                                                                                                           result = Scalar(0,0,0);
                                                                                                     }



                                                                                                     if(centroid_x.size() > 1)
                                                                                                     {

                                                                                                     line(result, Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Scalar(255,255,255), 5,8);


                                                                                                     }



                                                                                                     circle(current_image, Point(white_x3, white_y3), 5, Scalar(255,255,0), 2, 8, 0);

                                                                                                       cout<<"("<<white_x3<<","<<white_y3<<")"<<endl;


//(4) yellow

                                                                                      double white_x4 = 0;
                                                                                      double white_y4 = 0;
                                                                                      unsigned int white_count4 = 0;
                                                                                      centroid_x.clear();
                                                                                      centroid_y.clear();

                                                                                      int filter4[6]={20,11,255,188,255,0};


                                                                                      cvtColor(current_image, current_imageHSV, CV_BGR2HSV);
                                                                                      cvtColor(current_image, current_imageGRY, CV_BGR2HSV);// Convert colour to HSV

                                                                                    cv::inRange(current_imageHSV, cv::Scalar(filter4[1], filter4[3], filter4[5]), cv::Scalar(filter4[0], filter4[2], filter4[4]),current_imageBIN); //Filter HSV image to bet Buinary image

                                                                                      imshow("Camera", current_image);


                                                                                      for(unsigned int i=0; i<current_imageBIN.rows; i++)
                                                                                                     {
                                                                                                             for(unsigned int j=0; j<current_imageBIN.cols; j++)
                                                                                                             {
                                                                                                                     if(current_imageBIN.data[i*current_imageBIN.cols + j] > 200) // If colour is white
                                                                                                                     {
                                                                                                                             white_count4++; // Number of white pixels
                                                                                                                             white_x4 += j; //
                                                                                                                             white_y4 += i; //
                                                                                                                     }
                                                                                                             }
                                                                                                     }

                                                                                                     if(white_count4 > thresh)
                                                                                                     {
                                                                                                             white_x4 /= white_count4;
                                                                                                             white_y4 /= white_count4;
                                                                                                             centroid_x.push_back(white_x4);
                                                                                                             centroid_y.push_back(white_y4);
                                                                                                     }
                                                                                                     else
                                                                                                     {
                                                                                                             white_x4 = 0;
                                                                                                             white_y4 = 0;
                                                                                                             centroid_x.clear();
                                                                                                             centroid_y.clear();
                                                                                                           result = Scalar(0,0,0);
                                                                                                     }



                                                                                                     if(centroid_x.size() > 1)
                                                                                                     {

                                                                                                     line(result, Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Point(centroid_x[centroid_x.size()-1],centroid_y[centroid_y.size()-1]), Scalar(255,255,255), 5,8);


                                                                                                     }



                                                                                                     circle(current_image, Point(white_x4, white_y4), 5, Scalar(255,255,0), 2, 8, 0);

                                                                                                       cout<<"("<<white_x4<<","<<white_y4<<")"<<endl;


                                         imshow("current_image",current_image);


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



//ud[0]=287;
//ud[1]=212;

//ud[2]=379;
//ud[3]=178;

//ud[4]=353;
//ud[5]=235;

//ud[6]=320;
//ud[7]=137;

//z_d=0.32;


ud[0]=330;
ud[1]=189;

ud[2]=288;
ud[3]=149;

ud[4]=238;
ud[5]=188;

ud[6]=275;
ud[7]=227;

z_d=0.27;
//ud[0]=360;
//ud[1]=192;

//ud[2]=315;
//ud[3]=162;

//ud[4]=274;
//ud[5]=197;

//ud[6]=320;
//ud[7]=233;

//z_d=0.29;

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

   Zeta_PREV.at<double>(0,0)=Zeta.at<double>(0,0);Zeta_PREV.at<double>(1,0)=Zeta.at<double>(1,0);Zeta_PREV.at<double>(2,0)=Zeta.at<double>(2,0);
   Zeta_PREV.at<double>(3,0)=Zeta.at<double>(3,0);Zeta_PREV.at<double>(4,0)=Zeta.at<double>(4,0);Zeta_PREV.at<double>(5,0)=Zeta.at<double>(5,0);
   Zeta_PREV.at<double>(6,0)=Zeta.at<double>(6,0);Zeta_PREV.at<double>(7,0)=Zeta.at<double>(7,0);Zeta_PREV.at<double>(8,0)=Zeta.at<double>(8,0);
  Zeta_PREV.at<double>(9,0)=Zeta.at<double>(9,0);Zeta_PREV.at<double>(10,0)=Zeta.at<double>(10,0);Zeta_PREV.at<double>(11,0)=Zeta.at<double>(11,0);
Zeta_PREV.at<double>(12,0)=Zeta.at<double>(12,0);Zeta_PREV.at<double>(13,0)=Zeta.at<double>(13,0);Zeta_PREV.at<double>(14,0)=Zeta.at<double>(14,0);
Zeta_PREV.at<double>(15,0)=Zeta.at<double>(15,0);


    S_T = cv::Mat(8, 1, CV_64F, sd);
     err.at<double>(0,0) =  S.at<double>(0,0)-S_T.at<double>(0,0);
       err.at<double>(1,0) =  S.at<double>(1,0)-S_T.at<double>(1,0);
       err.at<double>(2,0) =  S.at<double>(2,0)-S_T.at<double>(2,0);
         err.at<double>(3,0) =  S.at<double>(3,0)-S_T.at<double>(3,0);
     err.at<double>(4,0) =  S.at<double>(4,0)-S_T.at<double>(4,0);
       err.at<double>(5,0) =  S.at<double>(5,0)-S_T.at<double>(5,0);
       err.at<double>(6,0) =  S.at<double>(6,0)-S_T.at<double>(6,0);
         err.at<double>(7,0) =  S.at<double>(7,0)-S_T.at<double>(7,0);


       Zeta.at<double>(0,0)= err.at<double>(0,0);
       Zeta.at<double>(1,0)= err.at<double>(1,0);
      Zeta.at<double>(2,0)=err.at<double>(2,0);
      Zeta.at<double>(3,0)=err.at<double>(3,0);
     Zeta.at<double>(4,0)=err.at<double>(4,0);
       Zeta.at<double>(5,0)=err.at<double>(5,0);
       Zeta.at<double>(6,0)=err.at<double>(6,0);
      Zeta.at<double>(7,0)=err.at<double>(7,0);



      Zeta.at<double>(8,0)= sd[0];
      Zeta.at<double>(9,0)= sd[1];
     Zeta.at<double>(10,0)= sd[2];
       Zeta.at<double>(11,0)= sd[3];
      Zeta.at<double>(12,0)= sd[4];
      Zeta.at<double>(13,0)= sd[5];
     Zeta.at<double>(14,0)= sd[6];
       Zeta.at<double>(15,0)= sd[7];

//cout<<"ZETA: "<<Zeta<<endl;

     //(1)
      //  W1.at<double>(0,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(1,0)= Zeta.at<double>(1,0);
      //  W1.at<double>(2,0)= Zeta.at<double>(2,0);
      //  W1.at<double>(3,0)= Zeta.at<double>(3,0);
      //  W1.at<double>(4,0)= Zeta.at<double>(4,0);
      //  W1.at<double>(5,0)= Zeta.at<double>(5,0);
      //  W1.at<double>(6,0)= Zeta.at<double>(6,0);
      //  W1.at<double>(7,0)= Zeta.at<double>(7,0);
      //  W1.at<double>(8,0)=  Zeta.at<double>(8,0);
      //  W1.at<double>(9,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(10,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(11,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(12,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(13,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(14,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(15,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(16,0)= Zeta.at<double>(0,0);
      //  W1.at<double>(17,0)= Zeta.at<double>(0,0);



      //  lay= W1.at<double>(0,0)* Zeta.at<double>(0,0)+W1.at<double>(1,0)* Zeta.at<double>(1,0)+
      //           W1.at<double>(2,0)*Zeta.at<double>(2,0)+W1.at<double>(3,0)* Zeta.at<double>(3,0)  + W1.at<double>(4,0)*Zeta.at<double>(4,0)+W1.at<double>(5,0)* Zeta.at<double>(5,0)+W1.at<double>(6,0)* Zeta.at<double>(6,0)+W1.at<double>(7,0)* Zeta.at<double>(7,0)+W1.at<double>(8,0)* Zeta.at<double>(8,0)+W1.at<double>(9,0)* Zeta.at<double>(9,0)+W1.at<double>(10,0)* Zeta.at<double>(10,0)+W1.at<double>(11,0)* Zeta.at<double>(11,0)+
      //           W1.at<double>(12,0)*Zeta.at<double>(12,0)+W1.at<double>(13,0)* Zeta.at<double>(13,0)  + W1.at<double>(14,0)*Zeta.at<double>(14,0)+W1.at<double>(15,0)* Zeta.at<double>(15,0)+W1.at<double>(16,0)* Zeta.at<double>(16,0)+W1.at<double>(17,0)* Zeta.at<double>(17,0);

      ////std::cout<<"lay"<<lay<<std::endl;

      //      double phii=0;
      //    phii=1/(1+exp(-lam*lay));
      ////   std::cout<<"PHI"<<phii<<std::endl;


      //      for(int i=0;i<st;i++)
      //        del_PHI.at<double>(0,i)=2*W1.at<double>(i,0)*lam*phii*(1-phii);


//(2) best out of 3: dt=0.01 (this was the one which was used last)
        double norme_z;
        sig.at<double>(0,0)=1; sig.at<double>(0,1)=0.75; sig.at<double>(0,2)=0.5; sig.at<double>(0,3)=0.0000000001;

        norme_z=Zeta.at<double>(0,0)*Zeta.at<double>(0,0)+Zeta.at<double>(1,0)*Zeta.at<double>(1,0)+Zeta.at<double>(2,0)*Zeta.at<double>(2,0)+Zeta.at<double>(3,0)*Zeta.at<double>(3,0)+Zeta.at<double>(4,0)*Zeta.at<double>(4,0)+Zeta.at<double>(5,0)*Zeta.at<double>(5,0)+Zeta.at<double>(6,0)*Zeta.at<double>(6,0)+Zeta.at<double>(7,0)*Zeta.at<double>(7,0);

        for(int i=0;i<4;i++)
          PHI.at<double>(0,i)=exp(-(norme_z)/(2*sig.at<double>(0,i)*sig.at<double>(0,i)));

        cout<<"phi"<<PHI<<endl;


           for(int i=0;i<4;i++)
             for(int j=0;j<st/2;j++)
               del_PHI.at<double>(i,j)= PHI.at<double>(0,i)*(0.5/(sig.at<double>(0,i)*sig.at<double>(0,i)))*2*Zeta.at<double>(j,0);


//(3)
//     //phi= [zeta(1,k)^2 zeta(2,k)^2 zeta(3,k)^2 .....zeta(8,k)^2]
//       for(int i=0;i<st/2;i++)
//         del_PHI.at<double>(i,i)=2*Zeta.at<double>(i,0);


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


                    for(int i=0;i<(st/2);i++)
                                          for(int j=0;j<6;j++)
                                            J_total.at<double>(i,j)=J_new.at<double>(i,j);

//                                        int h=(st/2)-1;
//                                        for(int j=0;j<6;j++)
//                                          J_total.at<double>(h,j)=Jp.at<double>(2,j);


             for (int i=0;i<st/2;i++)
                 for (int j=0;j<6;j++)
                     G.at<double>(i,j)=J_total.at<double>(i,j);

             cv::transpose(G,G_T);
             cv::transpose(Zeta,Zeta_T);
             cv::transpose(W,W_T);


             S_dot=(S_T-S_T_PREV)/dt;


               F.at<double>(0,0)=-S_dot.at<double>(0,0);
               F.at<double>(1,0)=-S_dot.at<double>(1,0);
               F.at<double>(2,0)=-S_dot.at<double>(2,0);
               F.at<double>(3,0)=-S_dot.at<double>(3,0);
               F.at<double>(4,0)=-S_dot.at<double>(4,0);
               F.at<double>(5,0)=-S_dot.at<double>(5,0);
               F.at<double>(6,0)=-S_dot.at<double>(6,0);
               F.at<double>(7,0)=-S_dot.at<double>(7,0);


               F.at<double>(8,0)=S_dot.at<double>(0,0);
               F.at<double>(9,0)=S_dot.at<double>(1,0);
               F.at<double>(10,0)=S_dot.at<double>(2,0);
               F.at<double>(11,0)=S_dot.at<double>(3,0);
               F.at<double>(12,0)=S_dot.at<double>(4,0);
               F.at<double>(13,0)=S_dot.at<double>(5,0);
               F.at<double>(14,0)=S_dot.at<double>(6,0);
               F.at<double>(15,0)=S_dot.at<double>(7,0);


              theta_dot=-0.5*Rinv*G_T*del_V;

              A=del_PHI*G*Rinv*G_T*del_PHI_T;
              e_h=Zeta_T*Qbar*Zeta+W_T*del_PHI*F-0.25*W_T*A*W;
              de_h=del_PHI*F-0.5*A*W;

              Zeta_dot=(Zeta-Zeta_PREV)/dt;

              temp=mu*cv::norm(e_h)*Zeta_T*Zeta_dot/(cv::norm(de_h*e_h)*cv::norm(Zeta));
              alpha=1+temp.at<double>(0,0);
              std::cout<<"alpha"<<alpha<<std::endl;

           sqer2=(Zeta.at<double>(2,0)* Zeta.at<double>(2,0)+Zeta.at<double>(0,0)* Zeta.at<double>(0,0)+Zeta.at<double>(1,0)* Zeta.at<double>(1,0)+Zeta.at<double>(3,0)* Zeta.at<double>(3,0)+Zeta.at<double>(4,0)* Zeta.at<double>(4,0)+Zeta.at<double>(5,0)* Zeta.at<double>(5,0)+Zeta.at<double>(6,0)* Zeta.at<double>(6,0)+Zeta.at<double>(7,0)* Zeta.at<double>(7,0))/8;

             W_dot=alpha*del_PHI*G*Rinv*G_T*Zeta;
              W=W+dt*W_dot;

              Zeta=Zeta+dt*(F+G*theta_dot);

              cout<<(Zeta.at<double>(0,0)+Zeta.at<double>(9,0))*px+u_ref<<endl;

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

              myfile15<<W.at<double>(0,0)<<endl;
              myfile16<<W.at<double>(1,0)<<endl;
              myfile17<<W.at<double>(2,0)<<endl;
              myfile18<<W.at<double>(3,0)<<endl;


              group.setJointValueTarget(joints);
              group.move();
              usleep(10000);

              joints = group.getCurrentJointValues();
              cout<<"joint angles "<<joints[1]<<joints[2]<<joints[3]<<endl;


              double cost=0;
              for(int i=0;i<6;i++)
                cost=cost+theta_dot.at<double>(i,0)*theta_dot.at<double>(i,0);

              for(int i=0;i<st/2;i++)
                cost=cost+Zeta.at<double>(i,0)*Zeta.at<double>(i,0);


              cout<<"COST: "<<cost<<endl;
              std::cout<<"squared error "<<sqer2<<std::endl;

              myfile1<<sqer2<<endl;
              myfile2<<cost<<endl;
              geometry_msgs::PoseStamped current_Pose;
              geometry_msgs::PoseStamped current_Pose_base;



          z_d_prev=z_d;

            for(int ij = 0; ij < 8; ij++)
            S_T_PREV.at<double>(ij,0)=sd[ij];


            if(t==T)
              break;
if(sqer2<=0.00009)
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
myfile16.close();
myfile17.close();
myfile18.close();

  std::cout<<"squared error2 "<<sqer2<<std::endl;
  return 0;
    }

