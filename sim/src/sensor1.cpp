#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <random>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
using namespace std;
using Eigen::MatrixXd;
using Eigen::Matrix;
// ofstream uwb_log("uwb_log.csv",ios::out);
// ofstream vision_log("vision_log.csv",ios::out);
// ofstream real_log("real_log.csv",ios::out);
// ofstream kalman_log("kalman_log.csv",ios::out);
ofstream uwb_log("uwb_log.csv",ios::out);
ofstream vision_log("vision_log.csv",ios::out);
ofstream real_log("real_log.csv",ios::out);
ofstream kalman_log("kalman_log.csv",ios::out);
//加噪，并进行卡尔曼滤波
double std_deviation(double a, double b,double c,double d)
{
    return sqrt((a-b)*(a-b)/2+(c-d)*(c-d)/2);
}
//模拟uwb噪音
class sensorLayer
{
public:
    sensorLayer()
    {
        sub_odom = nh.subscribe("odom",1000,&sensorLayer::onmsg_odom,this);
        pub_uwb = nh.advertise<nav_msgs::Odometry>("uwb_odom",1000);
        // Matrix<double,2,2> a;
        // a<< 1,1,
        //     0,1; 
        // Matrix<double,2,2> b;
        // b<< 1,0,
        //     0,2; 
        // Matrix<double,2,2> c;
        // c = a + b;
        // double tmp = 1;
        // Matrix<double,2,1> d;
        // d << tmp,tmp;
        // cout<<c<<endl;
        // cout<<"c.T"<<c.transpose()<<endl;
        // cout<<"a*b"<<a*b<<endl;
        // cout<<"a^-1"<<a.inverse()<<endl;
        // cout<<"c[1,0]"<<c(1,0)<<endl;;
        // cout<<"d"<<endl<<d<<endl;
        // cout<<d(0,0)<<","<<d(1,0)<<endl;
        // cout<<"ok"<<endl;

        x1_p = 0;
        y1_p = 0;
        I << 1,0,
             0,1;
        A = I;
        B = I;
        H = I;
        R << 1,0,
             0,1;
        Q << 1,0,
             0,1;
        P = I;
    }
    void onmsg_odom(const nav_msgs::Odometry& msg)
    {
        

        double x0 =   msg.pose.pose.position.x;
        double y0 =   msg.pose.pose.position.y;
        uwb_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
        uwb_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
        
        normal_distribution<double> distribution1(0,0.01);
        normal_distribution<double> distribution2(0,0.005);
        double x1 = msg.pose.pose.position.x + distribution1(generator1);
        double y1 = msg.pose.pose.position.y + distribution1(generator1);
        double x2 = msg.pose.pose.position.x + distribution2(generator2);
        double y2 = msg.pose.pose.position.y + distribution2(generator2);

        x << x1,y1;
        z << x2,y2;
        U << x1 - x1_p,y1-y1_p;

        x = A*x + B*U;
        P = A*P*A.transpose()+Q;
        k = P*H.transpose()*((H*P*H.transpose()+R).inverse());
        x = x + k*(z - H*x);
        P = (I - k*H)*P;

        // uwb_log<<setprecision(12)<<x1<<","<<setprecision(12)<<y1<<endl;
        // vision_log<<setprecision(12)<<x2<<","<<setprecision(12)<<y2<<endl;
        // real_log<<setprecision(12)<<x0<<","<<setprecision(12)<<y0<<endl;
        // kalman_log<<setprecision(12)<<x(0,0)<<","<<setprecision(12)<<x(1,0)<<endl;
        uwb_log<<setprecision(12)<<std_deviation(x1,x0,y1,y0)<<endl;
        vision_log<<setprecision(12)<<std_deviation(x2,x0,y2,y0)<<endl;
        real_log<<setprecision(12)<<x0<<","<<setprecision(12)<<y0<<endl;
        kalman_log<<setprecision(12)<<std_deviation(x(0,0),x0,x(1,0),y0)<<endl;
        uwb_odom.pose.pose.orientation.x = x1;
        uwb_odom.pose.pose.orientation.y = y1;
        pub_uwb.publish(uwb_odom);

        x1_p = x1;
        y1_p = y1;
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Publisher pub_uwb;
    default_random_engine generator1;
    default_random_engine generator2;
    nav_msgs::Odometry uwb_odom;
    nav_msgs::Odometry fusion_odom;
    Matrix<double,2,2> A;
    Matrix<double,2,2> B;
    Matrix<double,2,1> U;
    Matrix<double,2,2> H;
    Matrix<double,2,2> R;
    Matrix<double,2,2> Q;
    Matrix<double,2,2> P;
    Matrix<double,2,1> x;
    Matrix<double,2,1> z;
    Matrix<double,2,2> I;
    Matrix<double,2,2> k;
    double x1_p,y1_p;
};
int main(int argc,char ** argv)
{
    // uwb_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    // vision_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    // real_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    // kalman_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    uwb_log<<setprecision(12)<<"std_deviation"<<endl;
    vision_log<<setprecision(12)<<"std_deviation"<<endl;
    real_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    kalman_log<<setprecision(12)<<"std_deviation"<<endl;
    ros::init(argc,argv,"sensor");
    sensorLayer sensor;

    ros::spin();
}