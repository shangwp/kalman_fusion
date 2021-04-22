#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
//#include "PNPSolver.h"
#include <vector>
#include <iostream>




int ArucoCreat();
int ArucoDetect(cv::Mat image, cv::Mat tracker,cv::Point_<float> pot0);
int Track_aruco_line(cv::Mat track, int arucoid);
cv::Mat imageReduction2(cv::Mat &image, double kx, double ky);