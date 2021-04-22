//#include "stdafx.h"
#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <time.h>
#include "arucoform.h"
//#include "arucowaican.h"
#include <vector>
#include <iostream>
#include <fstream>
// #pragma comment( lib, "opencv_highgui430d.lib")
// #pragma comment( lib, "opencv_core430d.lib")
// #pragma comment( lib, "opencv_aruco430d.lib")
// #pragma comment( lib, "opencv_video430d.lib")
// #pragma comment( lib, "opencv_videoio430d.lib")
// #pragma comment( lib, "opencv_imgcodecs430d.lib")
// #pragma comment( lib, "opencv_calib3d430d.lib")
// #pragma comment( lib, "opencv_ccalib430d.lib")
// #pragma comment( lib, "opencv_imgproc430d.lib")


using namespace std;
using namespace cv;

clock_t start0, finish0;
vector<Point2f> vp2f(20);
bool captureOpen = false;
Mat frame;
VideoCapture capture;
String url = "rtsp://admin:admin123@192.168.61.66/cam/realmonitor?channel=1&subtype=0";

Size image_size = cv::Size(1280, 720);
Mat mapx = Mat(image_size, CV_32FC1);
Mat mapy = Mat(image_size, CV_32FC1);
Mat R = Mat::eye(3, 3, CV_32F);
cv::Matx33d intrinsic_matrix;   
cv::Vec4d distortion_coeffs;    

int fisheye_calibrate_img(Mat distort_img, Mat undistort_img)
{



	//Mat undistort_img;
	Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;

	intrinsic_mat.copyTo(new_intrinsic_mat);

	new_intrinsic_mat.at<double>(0, 0) *= 1;
	new_intrinsic_mat.at<double>(1, 1) *= 1;

	//new_intrinsic_mat.at<double>(0, 2) += 0;// = 0.5 * distort_img.cols;
	//new_intrinsic_mat.at<double>(1, 2) += 0;//= 0.5 * distort_img.rows;

	//Size image_size = distort_img.size();

	Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	
	//Mat undistort_img = distort_img.clone();
	cv::remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);

	
	//fisheye::undistortImage(distort_img, undistort_img, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
	//undistort_img = imageReduction1(undistort_img, 0.5, 0.5);
	return 0;
}


int main(int argc,char ** argv)
{
	int i=10;
    clock_t start,finish;
    double Times;
    start=clock();
    while(i--){
        cout<<i<<endl;
    };
	finish=clock();
	cout<<"start(ticks): "<<start<<endl;
    cout<<"finish(ticks): "<<finish<<endl;
    cout<<"CLOCKS_PER_SEC: "<<CLOCKS_PER_SEC<<endl;
    cout<<"Total Times(s)(CLOCKS_PER_SEC): "<<Times<<endl;

	ros::init(argc,argv,"aruco");
	init_form();
	/*waican();*/
    // ArucoCreat();
	//Mat img = cv::imread("acuro1.jpg", 1);
	// namedWindow("video", WINDOW_AUTOSIZE);
	//namedWindow("aruco", WINDOW_AUTOSIZE);
	//namedWindow("tracker", WINDOW_AUTOSIZE);
   // VideoCapture cap(0);
	
	//String url = "rtsp://admin:admin123@192.168.61.66/cam/realmonitor?channel=1&subtype=0";
	String url = "rtsp://admin:admin123@192.168.99.111/cam/realmonitor?channel=1&subtype=0";
	intrinsic_matrix = { 990.3282081767957, 0, 640.1259362409419,	0,  990.4995088268357, 363.9805854020683,0, 0, 1 };
	distortion_coeffs = { -0.121971, 0.0209669, -0.138727, 0.13801 };
	VideoCapture cap(url);
	//cap.open("rtsp://192.168.61.66/user=admin_password=admin123=1_stream=0.sdp?real_stream");
	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
	if (!cap.isOpened())
	{
		std::cerr << "Couldn't open capture." << std::endl;
		return -1;
	}
	Mat frame;
	Mat framecopy ;
	Mat trackeline;
	cv::Point_<float> pot;
	int frame5in1 = 0;
//	Ptr<Tracker> tracker =TrackerKCF::create();
//	Rect2d roi;
//	roi = selectROI("trackerkcf", framecopy);
//	tracker->init(framecopy, roi);
	//imshow("tracker",0);
	waitKey(5);
	while (1)
	{
		
		//finish0 = clock();
		//cout << "循环外时间：" << (double)(finish0 - start0) / CLOCKS_PER_SEC << endl;

		clock_t start2, finish2;
		start2 = clock();

		cap >> frame;
		if (!cap.read(frame))
			break;
		// frame5in1++;
		// if ((frame5in1 % 10) != 0)
		// {
		// continue;
		// }
		// frame5in1 = 0;

		//waitKey(1);
		if (frame.empty()) break;
		framecopy = frame.clone();
		frame = imageReduction2(frame, 0.5, 0.5);
		//cv::imshow("video", frame);;

		clock_t start1, finish1;
		start1 = clock();
		framecopy = imageReduction2(framecopy, 0.5, 0.5);
		finish1 = clock();
		cout << "缩放：" << (double)(finish1 - start1) / CLOCKS_PER_SEC << endl;

		clock_t start, finish;
		start = clock();
		fisheye_calibrate_img(framecopy, framecopy);
		finish = clock();
		cout << "鱼眼矫正：" << (double)(finish - start)/ CLOCKS_PER_SEC << endl;


		/*framecopy = imageReduction2(framecopy, 0.5, 0.5);*/
		//cv::imshow("correct", framecopy);	
		
		waitKey(1);
		framecopy = imageReduction2(framecopy, 0.5, 0.5);
		ArucoDetect(framecopy, trackeline, pot);
		//PictureToWorlf();
		//tracker->update(framecopy, roi);

		finish2 = clock();
		cout << "总时间：" << (double)(finish2 - start2) / CLOCKS_PER_SEC << endl;
	
		//start0 = clock();
		
	}

    //	Mat img = cv::imread("7.jpg", 1);
	 //ArucoDetect(img);
	return 0;
}



int main4()
{   
	
	/*ArucoCreat();*/
	return 0;
}
