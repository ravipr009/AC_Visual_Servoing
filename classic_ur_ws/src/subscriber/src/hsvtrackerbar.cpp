#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define CAMERA0 0


float data1[2][2];
float data2[2][2];
unsigned int Rclick, Lclick;

void HSVTuning()
{
	cout<<"HSV Tunning of Camera "<<endl;
	Mat frame, frameHSV, frameBIN;
	char key;
	int filter[6];

	namedWindow("Parameters"); // Windoe to display Parameter trackbar
	createTrackbar( "Hmax", "Parameters", &filter[0], 255, NULL);
	createTrackbar( "Hmin", "Parameters", &filter[1], 255, NULL);
	createTrackbar( "Smax", "Parameters", &filter[2], 255, NULL);
	createTrackbar( "Smin", "Parameters", &filter[3], 255, NULL);
	createTrackbar( "Vmax", "Parameters", &filter[4], 255, NULL);
	createTrackbar( "Vmin", "Parameters", &filter[5], 255, NULL);
	
	for(;;)
	{	
                frame=imread("view.jpg",IMREAD_COLOR);

		key = waitKey(5);
		cvtColor(frame, frameHSV, CV_BGR2HSV); // Convert colour to HSV
		
		inRange(frameHSV, Scalar(filter[1], filter[3], filter[5]), Scalar(filter[0], filter[2], filter[4]), frameBIN); //Filter HSV image to bet Buinary image
		
		if (key == 'q')
			break;
		imshow("Camera", frame);
		imshow("Camera HSV", frameHSV);
		imshow("Camera BIN", frameBIN);
	}
	destroyAllWindows();
}

//void OrientationOfCamera()
//{
//	cout<<"Check Orientation of Camera "<<endl;
//	Mat frame;
//	char key;
//	namedWindow("Camera 0"); // Window to display input RGB image
	
//	for(;;)
//	{
//		frame=imread("view.jpg",IMREAD_COLOR);
//		key = waitKey(5);
//		line(frame, Point(320, 0), Point(320, 480), Scalar(0,0,255), 1, 8);
//		if (key == 'q')
//			break;
//		imshow("Camera 0", frame);
//	}
//	destroyAllWindows();
//}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
        if  ( event == EVENT_RBUTTONDOWN )
        {
                cout << "mouse is R clicked - position (" << x << ", " << y << ")" << endl;
                data1[Rclick][0] = x;
                data1[Rclick][1] = y;
                Rclick++;
        }
 
        if  ( event == EVENT_LBUTTONDOWN )
        {
                cout << "mouse is L clicked - position (" << x << ", " << y << ")" << endl;
                data2[Lclick][0] = x;
                data2[Lclick][1] = y;
                Lclick++;
        }
}

bool Intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < 1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}



void OriginOfThread()
{
        unsigned short int i;
        Mat frame;
        Rclick = Lclick = 0;
        char key = 'r';
        namedWindow("Camera"); // Window to display input RGB image
	

        for(i=0; i<750; i++)
        {

                cout<<i<<endl;
                key = waitKey(10);
                if (key == 'q')
                        break;
                imshow("Camera", frame);

        }
        cout<<endl;
	


        Point2f r;


        setMouseCallback("Camera", CallBackFunc, NULL);

        for(;;)
        {

                imshow("Camera",frame);
                key = waitKey(0);
                if (key == 'q')
                        break;

		
                else if(key == 'c')
                {
                        Intersection(Point2f(data1[0][0],data1[0][1]), Point2f(data1[1][0],data1[1][1]), Point2f(data2[0][0],data2[0][1]), Point2f(data2[1][0],data2[1][1]), r);
                        cout<<r<<endl;
                        Rclick = 0;
                        Lclick = 0;
                }
        }
}

void LengthOfThread()
{

        Mat frame;

        float origin[2];
        cout<<" Enter center ";
        cin>>origin[0]>>origin[1];

        setMouseCallback("Camera", CallBackFunc, NULL);
        char key = 'r';
        for(;;)
        {
                Rclick = 0;

                imshow("Camera",frame);
                key = waitKey(0);
                if (key == 'q')
                        break;
                else if(key == 'n')
                {
                        cout<<sqrt((data1[0][0] - origin[0])*(data1[0][0] - origin[0]) + (data1[0][1] - origin[1])*(data1[0][1] - origin[1]))<<endl;
                }
        }
}

int main(int argc, char** argv)
{



	// OpenCV -------------------------

	Mat Frame[2];
	char key;
	

	// C++ -------------------------
	unsigned int i;
	
	
	
//	cout<<" ===Orientation Caliberation==="<<endl;	
//	for(i=0; i<2; i++)
	//{
	//	OrientationOfCamera(i, Camera[i]);
	//	cout<<" Orientation of camera "<<i<<" Done!!!"<<endl;
	//}
	
//	cout<<" ===Length Caliberation"<<endl;
//	cout<<" Which camera to use for origin calculation? ";
//	cin>>i;
//	OriginOfThread(Camera[i]);

//	cout<<" ===Distance Caliberation"<<endl;
//	LengthOfThread(Camera[i]);
	
	cout<<" ===HSV Caliberation==="<<endl;
	for(i=0; i<1; i++)
	{
                HSVTuning();
                cout<<" HSV Tunning of camera Done!!!"<<endl;
	}

	

	

	return 0;
}
