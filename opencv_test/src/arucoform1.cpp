//#include "stdafx.h"
#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
//#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>

#include "arucoform.h"
//#include "PNPSolver.h"
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;


//111
Mat R_CAMERA = (Mat_<double>(3, 3) << -0.9986088638749951, 0.05269333164451817, 0.001936437577686931, 0.05245682055782047, 0.9965118186685085, -0.064903599522269, -0.005349669826828756, -0.06471173042179658, -0.9978896597212342);
Mat T_CAMERA = (Mat_<double>(3, 1) << 194.2968591456466, -209.9583612923854, 377.1226711846767);

void calculateRealPoint(vector<Point2f> camera2d, Mat R, Mat t);

int dictionaryId = cv::aruco::DICT_6X6_50;
#define Row 200
#define Col 20
vector<vector<Point2f> > aruco_id_pot(Row, vector<Point2f>(Col));

int ArucoCreat()
{
	int markersX = 1;
	int markersY = 1;
	int markerLength = 700;
	int markerSeparation = 100;
	
	int margins = markerSeparation;

	int borderBits = 1;
	
	bool showImage = true;

	Size imageSize;

	imageSize.width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins;

	imageSize.height = markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	//Ptr<aruco::GridBoard> board = aruco::GridBoard::create(markersX, markersY, float(markerLength), float(markerSeparation), dictionary);
	Ptr<aruco::GridBoard> board = aruco::GridBoard::create(markersX, markersY, float(markerLength), float(markerSeparation), dictionary,9);
	// show created board
	Mat boardImage;
	board->draw(imageSize, boardImage, margins, borderBits);

	if (showImage) {
		imwrite(".\\Aruco1.jpg", boardImage);
		//imshow("board", boardImage);
		waitKey(0);
	}
	return 0;

}

cv::Mat imageReduction2(cv::Mat &image, double kx, double ky)
{
	int row = cvRound(image.rows * kx), col = cvRound(image.cols * ky);
	cv::Mat result(row, col, image.type());
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			int x = static_cast<int>((i + 1) / kx + 0.5) - 1;
			int y = static_cast<int>((j + 1) / ky + 0.5) - 1;
			result.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(x, y);
		}
	}
	return result;
}



int ArucoDetect(cv::Mat image, cv::Mat tracker, cv::Point_<float> pot0)
{
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
	//cv::Ptr<cv::aruco::DetectorParameters> params = aruco::DetectorParameters::create();
	//params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
	int id = 0; int idsinit = 0; 
	int idsno = 0;
	cv::Mat imageCopy;
	image.copyTo(imageCopy);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<std::vector<cv::Point2f>> cornersP;

	clock_t start0, finish0;
	start0 = clock();
	cv::aruco::detectMarkers(image, dictionary, corners, ids);
	finish0 = clock();
	cout << "检测靶标：" << (double)(finish0 - start0)/ CLOCKS_PER_SEC << endl;

	idsinit = ids.size();
	if (ids.size() > 0)
	{	
		cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
		idsno = 0;
	 
		while(idsno < idsinit) {
		id = ids[idsno];
		cout <<"aruco id: "<<id<<"    picture_x: "<< corners[idsno][0].x <<",picture_y: "<< corners[idsno][0].y << endl;
		pot0 = corners[idsno][0];
		corners[idsno][0].x = 2 * corners[idsno][0].x; corners[idsno][0].y = 2 * corners[idsno][0].y;

		clock_t start1, finish1;
		start1 = clock();
		calculateRealPoint(corners[idsno],R_CAMERA, T_CAMERA);
		finish1 = clock();
		cout << "计算坐标时间：" << (double)(finish1 - start1)/ CLOCKS_PER_SEC << endl;
		idsno++;
		//aruco_id_pot[id].push_back(pot0);
		
		//Track_aruco_line(imageCopy, id);
		
	   }
	}
	//imageCopy = imageReduction2(imageCopy, 0.5, 0.5);
	//cv::imshow("aruco", imageCopy);
	//cv::waitKey(1);

	return 0;
}

int Track_aruco_line(cv::Mat track,int arucoid)
{

	Point2f pold,pnew;
	Point2f zero = { 0,0 };
	auto sieze = aruco_id_pot[arucoid].size();
	if(sieze>21)
	{

		for (int i = 21; i > 0; i--)
		{
			pnew = aruco_id_pot[arucoid][i];
			pold = aruco_id_pot[arucoid][i-1];
			if (pold == zero) { break; }
			line(track,pold , pnew, Scalar(arucoid*20,255- arucoid * 20, arucoid * 10+5*i), 2, 8);
			//cout << "line: " << arucoid <<". "<<i<< ", oldx: " << pold.x << " ,newx: " << pnew.x << ",oldy: " << pold.y << ",newy: " << pnew.y << endl;
		}
		
		aruco_id_pot[arucoid].erase(aruco_id_pot[arucoid].begin());
		//imshow("tracker", tracker);
	//	waitKey(5);
	}
	//aruco_id_pot[arucoid].erase(aruco_id_pot[arucoid].begin());
	return 0;
}



	//float fx, fy, u0, v0, k1, k2, p1, p2, k3;
	double fx = 1952.7000; double fy = 1950.6000; double u0 = 1284.50000; double v0 = 727.7786;
	double k1 = -0.4283; double k2 = 0.1726; double p1 = 0.0000; double p2 = 0.0000; double k3 = 0.0000;

int  PictureToWorlf(cv::Mat image1, cv::Mat image2)
{	cv::Mat camMatrix;//�ڲ�������
	cv::Mat distCoeff;//����ϵ��

					  //camMatrix = (Mat_<float>(3, 3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);
					  //distCoeff = (Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

	double camD[9] = {
		fx, 0, u0,
		0, fy, v0,
		0, 0, 1 };
	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);

	//�������
	double distCoeffD[5] = { k1, k2, p1, p2, k3 };
	cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);
	undistort(image1, image2, camera_matrix, distortion_coefficients);
	
	Size image_size = image1.size();
	Size image_size_new = image1.size()/2;
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);



/*	initUndistortRectifyMap(camera_matrix, distortion_coefficients, Mat(),
		getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, image_size, 1, image_size_new, 0),
		image_size, CV_16SC2, mapx, mapy);
	cv::remap(image1, image2, mapx, mapy, INTER_LINEAR);*/
	return 0;
}


Mat K = (Mat_<double>(3, 3) <<  990.3282081767957, 0, 640.1259362409419,0,  990.4995088268357, 363.9805854020683,0, 0, 1 );
//Mat K = (Mat_<double>(3, 3) << 2014.457744916725, 0, 1280.413103402889, 0, 2009.799277282653, 720.8161591592659, 0, 0, 1);
Mat diffo = (Mat_<double>(5, 1) << -0.121971, 0.0209669, -0.138727, 0.13801,0.000000);
//Mat diffo = (Mat_<double>(5, 1) << -0.124788, -0.0180778, -0.465352, 0.948102, 0.000000);

float s = 0;

void calculateRealPoint(vector<Point2f> camera2d, Mat R, Mat t)
{

	// compute the error

	float error = 0, error_sum = 0, error_ave = 0, error_x = 0, error_y = 0, error_z = 0;
	Eigen::MatrixXd p(3, 1);
	Eigen::MatrixXd P(3, 1);

	cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	imagePoint.at<double>(0, 0) = camera2d[0].x;
	imagePoint.at<double>(1, 0) = camera2d[0].y;

	//compute the scal s
	cv::Mat tempMat, tempMat2;
	tempMat = R.inv()*K.inv()*imagePoint;
	tempMat2 = R.inv()*t;
	s = tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);
	//   cout<<"the scale is: "<<s<<endl;

	Mat wcPoint = R.inv() * (s * K.inv() * imagePoint - t);
	Point3f worldPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));

	int i = 0;
	imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	imagePoint.at<double>(0, 0) = camera2d[i].x;
	imagePoint.at<double>(1, 0) = camera2d[i].y;

	//compute the scale s
	tempMat = R.inv()*K.inv()*imagePoint;
	tempMat2 = R.inv()*t;
	s = tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);

	imagePoint.at<double>(0, 0) = camera2d[i].x;
	imagePoint.at<double>(1, 0) = camera2d[i].y;

	wcPoint = R.inv() * (s * K.inv() * imagePoint - t);
	Point3f worldPointLocal(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
	cout << "PNP: the estimate_x is:" << wcPoint.at<double>(0, 0) << " ,estimate_y :  " << wcPoint.at<double>(1, 0) << endl;// << " ,error_z :  " << wcPoint.at<double>(2, 0) << endl;

	/*
	error_x = abs(worldPointLocal.x);
	error_y = abs(worldPointLocal.y);
	error_z = abs(worldPointLocal.z);
	cout << "the error_x is:" << error_x << " ,error_y :  " << error_y << " ,error_z :  " << error_z << endl;*/
}