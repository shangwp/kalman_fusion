#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <random>
#include <ctime>
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
ofstream mean_log("mean_log.csv",ios::out);


//用于计算圆心坐标
default_random_engine generator1(time(0));
default_random_engine generator2(time(0)+1);
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
        R << 0.1,0,
             0,0.1;
        Q << 0.1,0,
             0,0.1;
        P = I;
        //标准差初始为0
        uwb_stdev = 0;
        vision_stdev = 0;
        kalman_stdev = 0;
        mean_stdev =0;
        num = 1;
    }
    void onmsg_odom(const nav_msgs::Odometry& msg)
    {
        

        double x0 =   msg.pose.pose.position.x;
        double y0 =   msg.pose.pose.position.y;
        uwb_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
        uwb_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
        
        normal_distribution<double> distribution1(0,0.1);
        normal_distribution<double> distribution2(0,0.1);
        double x1 = msg.pose.pose.position.x + distribution1(generator1);
        double y1 = msg.pose.pose.position.y + distribution1(generator1);
        double x2 = msg.pose.pose.position.x + distribution2(generator2);
        double y2 = msg.pose.pose.position.y + distribution2(generator2);
        if(x1 == x2)
        {
            cout<<"no"<<endl;
        }
        x << x1,y1;
        z << x2,y2;
        U << x1 - x1_p,y1-y1_p;

        x = A*x + B*U;
        P = A*P*A.transpose()+Q;
        k = P*H.transpose()*((H*P*H.transpose()+R).inverse());
        x = x + k*(z - H*x);
        P = (I - k*H)*P;

        uwb_log<<setprecision(12)<<x1<<","<<setprecision(12)<<y1<<","<<setprecision(12)<<std_deviation(x1,x0,y1,y0)<<","<<setprecision(12)<<x0<<endl;
        vision_log<<setprecision(12)<<x2<<","<<setprecision(12)<<y2<<","<<setprecision(12)<<std_deviation(x2,x0,y2,y0)<<","<<setprecision(12)<<x0<<endl;
        mean_log<<setprecision(12)<<(x1+x2)/2<<","<<setprecision(12)<<(y1+y2)/2<<","<<setprecision(12)<<std_deviation((x1+x2)/2,x0,(y1+y2)/2,y0)<<","<<setprecision(12)<<x0<<endl;
        real_log<<setprecision(12)<<x0<<","<<setprecision(12)<<y0<<endl;
        kalman_log<<setprecision(12)<<x(0,0)<<","<<setprecision(12)<<x(1,0)<<","<<setprecision(12)<<std_deviation(x(0,0),x0,x(1,0),y0)<<","<<setprecision(12)<<x0<<endl;
        // uwb_log<<setprecision(12)<<std_deviation(x1,x0,y1,y0)<<endl;
        // vision_log<<setprecision(12)<<std_deviation(x2,x0,y2,y0)<<endl;
        // real_log<<setprecision(12)<<x0<<","<<setprecision(12)<<y0<<endl;
        // kalman_log<<setprecision(12)<<std_deviation(x(0,0),x0,x(1,0),y0)<<endl;
        uwb_odom.pose.pose.orientation.x = x1;
        uwb_odom.pose.pose.orientation.y = y1;
        pub_uwb.publish(uwb_odom);

        //迭代计算标准差
        uwb_stdev = (num-1)*uwb_stdev/num + std_deviation(x1,x0,y1,y0)/num;
        vision_stdev = (num-1)*vision_stdev/num + std_deviation(x2,x0,y2,y0)/num;
        kalman_stdev = (num-1)*kalman_stdev/num + std_deviation(x(0,0),x0,x(1,0),y0)/num;
        mean_stdev = (num-1)*mean_stdev/num + std_deviation((x1+x2)/2,x0,(y1+y2)/2,y0)/num;
        if(num%1000 == 0)
        {
            cout<<"uwb:"<<uwb_stdev<<endl;
            cout<<"vision:"<<vision_stdev<<endl;
            cout<<"kalman:"<<kalman_stdev<<endl;
            cout<<"mean:"<<mean_stdev<<endl;
            cout<<"Q:"<<Q<<endl;
            cout<<"H:"<<H<<endl;
            cout<<"p:"<<P<<endl;
            cout<<"k:"<<k<<endl;
            cout<<"ok"<<endl<<endl;
        }
        x1_p = x1;
        y1_p = y1;
        num++;
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Publisher pub_uwb;

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

    //计算标准差
    double uwb_stdev;
    double vision_stdev;
    double kalman_stdev;
    double mean_stdev;
    int num;
};
int main(int argc,char ** argv)
{
    while(1)
    {
        double x1,y1,x2,y2;
        cout<<"x1,x2,x3,x4:"<<endl;
        cin>>x1>>y1>>x2>>y2;
        //cout<<x1<<","<<y1<<","<<x2<<","<<y2<<endl;
        double xr = (x2*x2/y2 -x1*x1/y1 + y2 -y1)/(x2/y2 - x1/y1);
        double yr = -x2/y2*(xr-x2) + y2;
        cout <<"xr = "<< xr <<", yr = " <<yr <<endl;
    }
    uwb_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y,"<<setprecision(12)<<"std_deviation,"<<setprecision(12)<<"real_x"<<endl;
    vision_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y,"<<setprecision(12)<<"std_deviation,"<<setprecision(12)<<"real_x"<<endl;
    real_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    kalman_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y,"<<setprecision(12)<<"std_deviation,"<<setprecision(12)<<"real_x"<<endl;
    mean_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y,"<<setprecision(12)<<"std_deviation,"<<setprecision(12)<<"real_x"<<endl;
    // uwb_log<<setprecision(12)<<"std_deviation"<<endl;
    // vision_log<<setprecision(12)<<"std_deviation"<<endl;
    // real_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    // kalman_log<<setprecision(12)<<"std_deviation"<<endl;
    ros::init(argc,argv,"sensor");
    //sensorLayer sensor;
    ros::spin();
}