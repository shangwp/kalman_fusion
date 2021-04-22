#include"stdafx.h"
#include<Windows.h>
#include <iostream>
#include <opencv2/eigen3/Eigen/Core>
#include <opencv2/eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/eigen3/Eigen/Core>
#include <opencv2/eigen3/Eigen/Geometry>
//#include <chrono>
#include <fstream>
#include <vector>




using namespace std;
using namespace cv;

//enum SolvePnPMethod {
//    SOLVEPNP_ITERATIVE   = 0,
//    SOLVEPNP_EPNP        = 1, //!< EPnP: Efficient Perspective-n-Point Camera Pose Estimation @cite lepetit2009epnp
//    SOLVEPNP_P3P         = 2, //!< Complete Solution Classification for the Perspective-Three-Point Problem @cite gao2003complete
//    SOLVEPNP_DLS         = 3, //!< A Direct Least-Squares (DLS) Method for PnP  @cite hesch2011direct
//    SOLVEPNP_UPNP        = 4, //!< Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation @cite penate2013exhaustive
//    SOLVEPNP_AP3P        = 5, //!< An Efficient Algebraic Solution to the Perspective-Three-Point Problem @cite Ke17
//    SOLVEPNP_IPPE        = 6, //!< Infinitesimal Plane-Based Pose Estimation @cite Collins14 \n
//    //!< Object points must be coplanar.
//    SOLVEPNP_IPPE_SQUARE = 7,
double eDisR, eDist;

Eigen::MatrixXd trueR(3,3);
Eigen::MatrixXd truet(3,1);

void calculateDis(Eigen::MatrixXd R, Eigen::MatrixXd t);

int method=0; //   2/5/7
string methodName;
string txt=".txt";
string dataset="truth";
//string inputFileDir="/home/flk/SlovePnP/pnpData/experiment_12.16/rowdata/distur2-5_rawdata/"+dataset+"/";
//string outputFileDir="/home/flk/SlovePnP/pnpData/experiment_12.16/rowdata/distur2-5_rawdata/"+dataset+"/results/";
string inputFileDir = "C:/Users/lenovo/Desktop/graduation/location/aurcotest/aurcotest/" + dataset + "/";
string outputFileDir= "C:/Users/lenovo/Desktop/graduation/location/aurcotest/aurcotest/"+dataset+"/results/";
vector<Point3f> worldP3Part;
vector<Point2f> cameraP2Part;
int countt=0;
float s=0;
//Mat K = ( Mat_<double> ( 3,3 ) << 4838.4, 0, 1280, 0, 3628.8, 720, 0, 0, 1 );
Mat K = (Mat_<double>(3, 3) << 990.3282081767957, 0, 640.1259362409419, 0, 990.4995088268357, 363.9805854020683, 0, 0, 1);
//Mat K = (Mat_<double>(3, 3) << 2014.457744916725, 0, 1280.413103402889, 0, 2009.799277282653, 720.8161591592659, 0, 0, 1);
//Mat diffo = (Mat_<double>(5, 1) << -0.4283, 0.1726, 0, 0, 0);
Mat diffo = (Mat_<double>(5, 1) << -0.121971, 0.0209669, -0.138727, 0.13801, 0.000000);
void calculateError(vector<Point3f> world3d, vector<Point2f> camera2d, Mat R, Mat t, string methodNmae_);
void calculateRealPoint(vector<Point2f> camera2d, Mat R, Mat t);
int maxy=98000;

int waican( )
{
    for(method;method<7;method++)
    {
        if(method==1||method==2||method==3||method==4||method==5||method==7||method==6)
            continue;
        else
        {
            switch (method) {
                case 0:
                    methodName="ITERATIVE";
                    break;
                case 1:
                    methodName="EPNP";
                    break;
                case 2:
                    methodName="P3P";
                    break;
                case 3:
                    methodName="DLS";
                    break;
                case 4:
                    methodName="UPNP";
                    break;
                case 5:
                    methodName="AP3P";
                    break;
                case 6:
                    methodName="IPPE";
                    break;
            }
            cout<<"the method is: "<<methodName<<endl;

            vector<Point3f> pts_3d;
            vector<Point2f> pts_2d;
            Point2f midleP;
            Point3f midleP3;
            double num1, num2, num3;

            ifstream fu(inputFileDir+"u_origin.txt");
            ifstream fv(inputFileDir+"v_origin.txt");
            ifstream fx(inputFileDir+"x_origin.txt");
            ifstream fy(inputFileDir+"y_origin.txt");
            ifstream fz(inputFileDir+"z_origin.txt");

            assert(fu.is_open()); //若失败,则输出错误消息,并终止程序运行
			cout << "the method is: " << methodName << endl;
			int i = 0;
			while (!fu.eof()&&!fv.eof()&&(i<29))
            {
                fu>>num1;
                fv>>num2;
                midleP.x=num1;
                midleP.y=num2;
                pts_2d.push_back(midleP);
				i++;

            }
            cout<<"size of pts_2d: "<<pts_2d.size()<<endl;
			i = 0;
            while (!fx.eof()&&!fy.eof()&&!fz.eof() && (i<29))
            {
				i++;
                fx>>num1;
                fy>>num2;
                fz>>num3;
                midleP3.x=num1;
                midleP3.y=num2;
                midleP3.z=num3;
                pts_3d.push_back(midleP3);
            }
            cout<<"size of pts_3d:"<<pts_3d.size()<<endl;
//
//            cout<<pts_2d[35]<<endl<<pts_3d[35]<<endl;


            // add 0.01 distur
       /*     srand( (unsigned)time( NULL ) );
            for(int i=0;i<pts_3d.size();i++)
            {
                pts_3d[i].x=pts_3d[i].x+(50*((rand()%100/(double)101)-0.5));
                pts_3d[i].y=pts_3d[i].y+(50*((rand()%100/(double)101)-0.5));
                pts_3d[i].z=pts_3d[i].z+(50*((rand()%100/(double)101)-0.5));
                pts_2d[i].x=pts_2d[i].x;
                pts_2d[i].y=pts_2d[i].y;
            }
			*/
            Mat r, t, RR;

            for(int i=0;i<=5;i++)
            {
                worldP3Part.push_back(pts_3d[i]);
                cameraP2Part.push_back(pts_2d[i]);
            }

//    copy(pts_3d.begin(),pts_3d.begin()+24, worldP3Part.begin() );
//    copy(pts_2d.begin(), pts_2d.begin()+24,cameraP2Part.begin());
//            cout<<"the part of world points"<< worldP3Part.size()<<endl;
//            cout<<"worldP3Part[20]"<<worldP3Part[20]<<endl;
//            cout<<"cameraP2Part[20]"<<cameraP2Part[20]<<endl;
            //   return -1;
//
            // solve the pnp
           // cout<<"method: "<<method<<endl;
            solvePnP ( worldP3Part, cameraP2Part, K, Mat(), r, t, false ,method); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
			// solvePnP(worldP3Part, cameraP2Part, K, diffo, r, t, false, method);

            Rodrigues ( r, RR); // r为旋转向量形式，用Rodrigues公式转换为矩阵


			calculateError(pts_3d,pts_2d, RR,t,methodName);//图像坐标反投影到世界坐标
			//calculateRealPoint(pts_2d, RR, t);//图像坐标反投影到世界坐标
            int m=0, n=180;
        }
    }
	return 0;
}

void calculateError(vector<Point3f> world3d, vector<Point2f> camera2d, Mat R, Mat t, string methodName_)
{
	//虚拟相机的外参
    trueR<<0.9961980177959597, 0.0196, -0.0849,
            0.0872, -0.2242505587701737, -0.9707,
            0, 0.9706297846156308, -0.2251093986976954;

    truet<<0,
            5261.946922898293,
            1208.48645468081;

    Eigen::MatrixXd RE(3,3);
    Eigen::MatrixXd tE(3,1);
    Eigen::MatrixXd KE(3,3);

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            RE(i,j)=R.at<double>(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            KE(i,j)=K.at<double>(i,j);
        }
    }

    for(int i2=0;i2<3;i2++)
    {
        tE(i2,0)=t.at<double>(i2,0);
    }

    calculateDis(RE,tE);

//    cout<<"RE:  "<<RE<<endl;
//    cout<<"tE:  "<<tE<<endl;
//    cout<<"KE:  "<<KE<<endl;

    // compute the error

    float error=0,error_sum=0,error_ave=0,error_x=0,error_y=0,error_z=0, error_x_sum=0, error_y_sum=0,error_z_sum=0, error_x_ave=0, error_y_ave=0,error_z_ave=0;
    Eigen::MatrixXd p(3,1);
    Eigen::MatrixXd P(3,1);

    cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
    imagePoint.at<double>(0, 0) = camera2d[20].x;
    imagePoint.at<double>(1, 0) = camera2d[20].y;

    //compute the scal s
    cv::Mat tempMat, tempMat2;
    tempMat=R.inv()*K.inv()*imagePoint;
    tempMat2=R.inv()*t;
    s=tempMat2.at<double>(2,0);
    s/=tempMat.at<double>(2,0);
 //   cout<<"the scale is: "<<s<<endl;

    Mat wcPoint = R.inv() * (s * K.inv() * imagePoint - t);
    Point3f worldPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
 //   cout<<"the world point for i=20 is: "<<worldPoint<<endl;


    ofstream errorf(outputFileDir+methodName_+"_"+"error.txt");//+methodName_+"/"
//    ofstream plotErrorf(outputFileDir+"plotError.txt");
//    ofstream singleErrorf(outputFileDir+"singleError.txt");
    ofstream xErrorf(outputFileDir+methodName_+"-"+"xError.txt");
    ofstream yErrorf(outputFileDir+methodName_+"-"+"yError.txt");
    ofstream distancef(outputFileDir+methodName_+"_"+"distance.txt");

    errorf<<"the method is: "<<methodName_<<"\n"<<"the dataset is:"<<dataset<<"\n"<<"the disturb is -25mm~25mm for world coordinate and no disturb for image points""\n"<<"\n";
    errorf<<"the estimated R is: "<<R<<"\n"<<"the estimated t is: "<<t<<"\n";
    errorf<<"the euclidean distance for R is:"<<eDisR<<"\n"<<"the euclidean distance for t is:"<<eDist<<"\n"<<"\n";

    vector<float> verror_x;
    vector<float> verror_y;
    vector<float> verror_z;


 //   for(int dis=20000; dis<=maxy; dis=dis+2000)
 //   {
        for(int i=0; i<world3d.size(); i++)
        {
  //          if((world3d[i].y-dis)/dis<=0.01)
  //          {
                countt=countt+1;
 //               cout<<"d="<<dis<<endl;

                imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
                imagePoint.at<double>(0, 0) = camera2d[i].x;
                imagePoint.at<double>(1, 0) = camera2d[i].y;

                //compute the scale s
                tempMat=R.inv()*K.inv()*imagePoint;
                tempMat2=R.inv()*t;
                s=tempMat2.at<double>(2,0);
                s/=tempMat.at<double>(2,0);
//                cout<<"the scale is: "<<s<<endl;

                imagePoint.at<double>(0, 0) = camera2d[i].x;
                imagePoint.at<double>(1, 0) = camera2d[i].y;

                wcPoint = R.inv() * (s * K.inv() * imagePoint - t);
                Point3f worldPointLocal(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));

                //error_x=abs(worldPointLocal.x-world3d[i].x);
               // error_y=abs(worldPointLocal.y-world3d[i].y);
               // error_z=abs(worldPointLocal.z-world3d[i].z);
			   error_x=abs(worldPointLocal.x);
			   error_y=abs(worldPointLocal.y);
			   error_z=abs(worldPointLocal.z);

//                singleErrorf<<"index:"<<i<<" x:"<<worldPointLocal.x<<" y:"<<worldPointLocal.y<<" z:"<<worldPointLocal.z<<" error_x:"<<error_x<<" error_y:"<<error_y<<endl;

                distancef<<world3d[i].y<<"\n";
                xErrorf<<error_x<<"\n";
                yErrorf<<error_y<<"\n";

//                error_x_sum=error_x+error_x_sum;
//                error_y_sum=error_y+error_y_sum;
//                error_z_sum=error_z+error_z_sum;

//                cout<<"the predect world point"<<i<<"is: "<<worldPointLocal<<endl;
//                cout<<"the real world poit"<<i<<"is: "<<world3d[i]<<endl;
//            }
//       }
//        error_x_ave=error_x_sum/countt;
//        error_y_ave=error_y_sum/countt;
//        error_z_ave=error_z_sum/countt;
//
//        verror_x.push_back(error_x_ave);
//        verror_y.push_back(error_y_ave);
//        verror_z.push_back(error_z_ave);

//        errorf<<"the distance is: "<< dis<<"\n"<<"the average error for x is: "<<error_x_ave<<"\n"<<"the average error for y is: "<<error_y_ave<<"\n"<<"the average error for z is: "<<error_z_ave<<"\n"<<"\n";
        countt = 0;
//        error_sum=0;
//        error_x_sum=0;
//        error_y_sum=0;
//        error_z_sum=0;
    }
//    for(int dis=20000; dis<92000; dis=dis+2000)
//    {
//        if(dis==20000)
//        {
//            plotErrorf<<"distance: "<<"\n";
//        }
//        plotErrorf<<dis<<"\n";
//        if(dis==92000)
//        {
//            plotErrorf<<"\n"<<"\n";
//        }
//    }
//
//    for(int i=0; i<verror_x.size(); i++)
//    {
//        if(i==0)
//        {
//            plotErrorf<<"errorx:"<<"\n";
//        }
//        plotErrorf<<verror_x[i]<<"\n";
//        if(i==verror_x.size())
//       {
//            plotErrorf<<"\n"<<"\n";
//        }
//    }
//
//    for(int i=0; i<verror_y.size(); i++)
//    {
//        if(i==0)
//        {
//            plotErrorf<<"errory:"<<"\n";
//        }
//        plotErrorf<<verror_y[i]<<"\n";
//        if(i==verror_y.size())
//        {
//            plotErrorf<<"\n"<<"\n";
//        }
//    }
//
//    for(int i=0; i<verror_z.size(); i++)
//    {
//        if(i==0)
//        {
//            plotErrorf<<"errorz:"<<"\n";
//        }
//        plotErrorf<<verror_z[i]<<"\n";
//        if(i==verror_z.size())
//        {
//            plotErrorf<<"\n"<<"\n";
//        }
//    }

//    plotErrorf.close();
    errorf.close();
    xErrorf.close();
    yErrorf.close();
    distancef.close();
//    singleErrorf.close();
}


//calculate the euclidean distance
void calculateDis(Eigen::MatrixXd R, Eigen::MatrixXd t)
{
    double sum=0;
    Eigen::MatrixXd trueR(3,3);
    Eigen::MatrixXd truet(3,1);

    trueR<<0.9961980177959597, 0.08711778296237149, 3.508734154855642e-05,
            0.01964508855730578, -0.2242505587701737, -0.9743334939264073,
            -0.08487390550090182, 0.9706297846156308, -0.2251093986976954;

    truet<<-1.495050585197469,
            5261.946922898293,
            1208.48645468081;

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            sum=sum+(R(i,j)-trueR(i,j))*(R(i,j)-trueR(i,j));
        }
    }
    eDisR=pow(sum,0.5);
    sum=0;
    for(int i=0;i<3;i++)
    {
        sum=sum+pow((t(i,0)-truet(i,0)),2);
    }
    eDist=pow(sum,0.5);
}

void calculateRealPoint1(vector<Point2f> camera2d, Mat R, Mat t)
{
	
	// compute the error

	float error = 0, error_sum = 0, error_ave = 0, error_x = 0, error_y = 0, error_z = 0;
	Eigen::MatrixXd p(3, 1);
	Eigen::MatrixXd P(3, 1);

	cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	imagePoint.at<double>(0, 0) = camera2d[20].x;
	imagePoint.at<double>(1, 0) = camera2d[20].y;

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


		error_x = abs(worldPointLocal.x);
		error_y = abs(worldPointLocal.y);
		error_z = abs(worldPointLocal.z);
		cout << "the error_x is:" << error_x  << " ,error_y :  " << error_y << " ,error_z :  " << error_z << endl;
		}


//int fisheye_calibrate_img()
//{
//	ofstream fout("caliberation_result.txt");  /**    保存定标结果的文件     **/
//
//	 /************************************************************************
//	  读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
//	  *************************************************************************/
//	/*cout <<"开始提取角点………………" << endl;*/
//	cout << "start" << endl;
//	int image_count = 26;                    /****    图像数量     ****/
//	Size board_size = Size(11, 8);            /****    定标板上每行、列的角点数       ****/
//	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
//	vector<vector<Point2f>>  corners_Seq;    /****  保存检测到的所有角点       ****/
//	vector<Mat>  image_Seq;
//	int successImageNum = 0;                /****   成功提取角点的棋盘图数量    ****/
//	string inputimagefolder = "C:/Users/lenovo/Desktop/毕业设计/视觉定位/aurcotest/aurcotest/1090/";
//	int count = 0;
//	for (int i = 0; i != image_count; i++)
//	{
//		cout << "Frame #" << i + 1 << "..." << endl;
//		string imageFileName;
//		std::stringstream StrStm;
//		StrStm << i + 1;
//		StrStm >> imageFileName;
//		imageFileName += ".jpg";
//
//		cv::Mat image = imread(inputimagefolder + "out" + imageFileName);
//		/* 提取角点 */
//		Mat imageGray;
//		cvtColor(image, imageGray, CV_RGB2GRAY);
//		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
//			CALIB_CB_FAST_CHECK);
//		if (!patternfound)
//		{
//			cout << "找不到角点，需删除图片文件" << imageFileName << "重新排列文件名，再次标定" << endl;
//			getchar();
//			exit(1);
//		}
//		else
//		{
//			/* 亚像素精确化 */
//			cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//			/* 绘制检测到的角点并保存 */
//			Mat imageTemp = image.clone();
//			for (int j = 0; j < corners.size(); j++)
//			{
//				circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
//			}
//			string imageFileName;
//			std::stringstream StrStm;
//			StrStm << i + 1;
//			StrStm >> imageFileName;
//			imageFileName += "_corner.jpg";
//			imwrite(imageFileName, imageTemp);
//			cout << "Frame corner#" << i + 1 << "...end" << endl;
//
//			count = count + corners.size();
//			successImageNum = successImageNum + 1;
//			corners_Seq.push_back(corners);
//		}
//		image_Seq.push_back(image);
//	}
//	cout << "角点提取完成！\n";
//	/************************************************************************
//	摄像机定标
//	*************************************************************************/
//	/*cout << "开始定标………………" << endl;*/
//	cout << "start calibrate" << endl;
//	Size square_size = Size(20, 20);
//	vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/
//
//	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   保存提取的所有角点   *****/
//	vector<int>  point_counts;
//	/* 初始化定标板上角点的三维坐标 */
//	for (int t = 0; t < successImageNum; t++)
//	{
//		vector<Point3f> tempPointSet;
//		for (int i = 0; i < board_size.height; i++)
//		{
//			for (int j = 0; j < board_size.width; j++)
//			{
//				/* 假设定标板放在世界坐标系中z=0的平面上 */
//				Point3f tempPoint;
//				tempPoint.x = i*square_size.width;
//				tempPoint.y = j*square_size.height;
//				tempPoint.z = 0;
//				tempPointSet.push_back(tempPoint);
//			}
//		}
//		object_Points.push_back(tempPointSet);
//	}
//	for (int i = 0; i < successImageNum; i++)
//	{
//		point_counts.push_back(board_size.width*board_size.height);
//	}
//	/* 开始定标 */
//	Size image_size = image_Seq[0].size();
//	cv::Matx33d intrinsic_matrix;    /*****    摄像机内参数矩阵    ****/
//	cv::Vec4d distortion_coeffs;     /* 摄像机的4个畸变系数：k1,k2,k3,k4*/
//	std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
//	std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
//	int flags = 0;
//	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
//	flags |= cv::fisheye::CALIB_CHECK_COND;
//	flags |= cv::fisheye::CALIB_FIX_SKEW;
//	fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
//	cout << "定标完成！\n";
//
//	/************************************************************************
//	对定标结果进行评价
//	*************************************************************************/
//	/*cout << "开始评价定标结果………………" << endl;*/
//	cout << "start evaluate result" << endl;
//	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
//	double err = 0.0;                        /* 每幅图像的平均误差 */
//	vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/
//
//	cout << "每幅图像的定标误差：" << endl;
//	cout << "每幅图像的定标误差：" << endl << endl;
//	for (int i = 0; i < image_count; i++)
//	{
//		vector<Point3f> tempPointSet = object_Points[i];
//		/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
//		fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs);
//		/* 计算新的投影点和旧的投影点之间的误差*/
//		vector<Point2f> tempImagePoint = corners_Seq[i];
//		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
//		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
//		for (size_t i = 0; i != tempImagePoint.size(); i++)
//		{
//			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
//			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
//		}
//		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
//		total_err += err /= point_counts[i];
//		std::cout <<"第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
//		fout << "第" << i + 1 << "幅图像的平均误差:" << err << "像素" << endl;
//	}
//	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
//	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
//	cout << "评价完成！" << endl;
//
//	/************************************************************************
//	保存定标结果
//	*************************************************************************/
//	/*cout << "开始保存定标结果………………" << endl;*/
//	cout << "start reserve result" << endl;
//	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
//
//	fout << "相机内参数矩阵：" << endl;
//	fout << intrinsic_matrix << endl;
//	fout << "畸变系数：\n";
//	fout << distortion_coeffs << endl;
//	for (int i = 0; i < image_count; i++)
//	{
//		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
//		fout << rotation_vectors[i] << endl;
//
//		/* 将旋转向量转换为相对应的旋转矩阵 */
//		Rodrigues(rotation_vectors[i], rotation_matrix);
//		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
//		fout << rotation_matrix << endl;
//		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
//		fout << translation_vectors[i] << endl;
//	}
//	cout << "完成保存" << endl;
//	fout << endl;
//
//
//	/************************************************************************
//	显示定标结果
//	*************************************************************************/
//	Mat mapx = Mat(image_size, CV_32FC1);
//	Mat mapy = Mat(image_size, CV_32FC1);
//	Mat R = Mat::eye(3, 3, CV_32F);
//
//
//
//
//	/************************************************************************
//	测试一张图片
//	*************************************************************************/
//	if (1)
//	{
//		//cout<<"TestImage ..."<<endl;
//		//Mat testImage = imread("a.jpg",1);
//		//fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,
//		//    getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0),image_size,CV_32FC1,mapx,mapy);
//		//Mat t = testImage.clone();
//		//cv::remap(testImage,t,mapx, mapy, INTER_LINEAR);
//
//		//imwrite("TestOutput.jpg",t);
//		//cout<<"保存结束"<<endl;
//
//		cout << "TestImage ..." << endl;
//		Mat distort_img = imread("a.jpg", 1);
//		Mat undistort_img;
//		Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;
//
//		intrinsic_mat.copyTo(new_intrinsic_mat);
//		//调节视场大小,乘的系数越小视场越大
//		new_intrinsic_mat.at<double>(0, 0) *= 0.9;
//		new_intrinsic_mat.at<double>(1, 1) *= 0.9;
//		//调节校正图中心，建议置于校正图中心
//		new_intrinsic_mat.at<double>(0, 2) += 0;// = 0.5 * distort_img.cols;
//		new_intrinsic_mat.at<double>(1, 2) += 0;//= 0.5 * distort_img.rows;
//
//		fisheye::undistortImage(distort_img, undistort_img, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
//		imwrite("output.jpg", undistort_img);
//		cout << "保存结束" << endl;
//		
//		cv::imshow("correct", undistort_img);
//		waitKey(100000);
//	}
//
//
//	return 0;
//}








