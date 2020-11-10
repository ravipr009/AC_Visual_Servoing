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





using namespace cv; //for smooth working of MAT
using namespace std;


Mat frameHSV;
Mat frameBIN;
Mat frame;


int filter[6] = {255, 68, 255, 174, 255, 88};
int thresh = 15;

#define SPACE_KEY (32)



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{cv_bridge::CvImagePtr cv_ptr;


try
{

std::cout << "in function" << std::endl;
cv_ptr=cv_bridge::toCvCopy(msg);
frame = cv_bridge::toCvShare(msg)->image;



}


catch(cv_bridge::Exception& e)
{ROS_ERROR("could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
}


  imshow("image",frame);
  cv::waitKey(10);
  int key= cv::waitKey(10);




  ////to save the figure as view.jpg in home when space key is pressed
  //if( key==SPACE_KEY)
  //{static int count=0;
  //ROS_ASSERT(cv::imwrite("view.jpg",frame));
  //}


  frame = cv_bridge::toCvShare(msg)->image;
  Mat result(frame.rows,frame.cols, CV_8UC3, Scalar(0,0,0));


  char key2;
  unsigned int i,j=0;
  unsigned int t = 0;

          unsigned int white_x = 0;  //sumation of all x-coordinated of white pixels
          unsigned int white_y = 0;  //sumation of all y-coordinated of white pixels

          unsigned int white_count =  0; ////Count of all white pixels

          vector <unsigned int> centroid_x;
          vector <unsigned int> centroid_y;

  for(;;)
  {
                  white_x = 0;
                  white_y = 0;
                  white_count = 0;
                  //imshow("frame",frame);




                  cvtColor(frame, frameHSV, COLOR_BGR2HSV); // Convert colour to HSV
                  inRange(frameHSV, Scalar(filter[1], filter[3], filter[5]), Scalar(filter[0], filter[2], filter[4]),frameBIN); //Filter HSV image to bet Buinary image




                     for(unsigned int i=0; i<frameBIN.rows; i++)
                  {
                          for(unsigned int j=0; j<frameBIN.cols; j++)
                          {
                                  if(frameBIN.data[i*frameBIN.cols + j] > 200) // If colour is white
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



                  circle(frame, Point(white_x, white_y), 5, Scalar(0,255,2), 2, 8, 0);
                  //line(frame, Point(320,0), Point(320,480), Scalar(255,255,255), 1, 8, 0);
                  //if (cc==9)
                    cout<<"("<<white_x<<","<<white_y<<")"<<endl;

                 key2 = waitKey(30);
                  if (key2 == 'q')
                         break;
                  imshow("Camera", frame);
                  imshow("Camera HSV", frameHSV);
                  imshow("Camera BIN", frameBIN);
                  imshow("Centroid Trajectory",result);




               }


  frame.release();
  frameHSV.release();
  frameBIN.release();

}





int main(int argc, char **argv)
{

//  ros::init(argc, argv, "sub_pcl");
//  ros::NodeHandle nh;
//  ros::Subscriber sub = nh.subscribe<PointCloud>("camera1/depth/points", 1, callback);
// ros::spin();
  ros::init(argc, argv, "image_listener");
ros::NodeHandle nh;
cv::namedWindow("view");
cv::startWindowThread();
image_transport::ImageTransport it(nh);
image_transport::Subscriber sub=it.subscribe("camera1/depth/image_raw",1,imageCallback);
ros::spin();

//pub=it.advertise("camera/image_processed",1);






//ros::Rate loop_rate(50);
//while(ros::ok()){
//  ros::spinOnce();
//  loop_rate.sleep();
//}

//spinner.start();
cv::destroyWindow("view");


}
