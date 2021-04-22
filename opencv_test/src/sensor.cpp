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

ofstream kalman_log("kalman_log.csv",ios::out);
//ofstream mean_log("mean_log.csv",ios::out);

// //用于循迹实验
// //首先无误差循迹
// default_random_engine generator1(time(0));
// default_random_engine generator2(time(0)+1);

double delta =  0.096; //uwb位置校正
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
	    cout<<"init"<<endl;
        sub_odom= nh.subscribe("odom",1000,&sensorLayer::onmsg_odom,this);
        sub_uwb_odom = nh.subscribe("uwb_odom",1000,&sensorLayer::onmsg_uwb_odom,this);
        sub_vision= nh.subscribe("vision",1000,&sensorLayer::onmsg_vision,this);
        pub_uwbe = nh.advertise<nav_msgs::Odometry>("uwbe_odom",1000);
        pub_vision = nh.advertise<nav_msgs::Odometry>("vision_odom",1000);
        pub_fusion = nh.advertise<nav_msgs::Odometry>("fusion_odom",1000);

        //初始化kalman filter
        X_p << 0,0;

        I << 1,0,
             0,1;
        A = I;
        B = I;
        H = I;

        Q << 0.001,0,
             0,0.001;
        R << 0.000163448368257901,0,
             0,0.000163448368257901;
        P_p = I;

        //初始化 flag
        uwb_flag = 0;
        camera_flag = 0;
    }

    void kalman_filter(int x)
    {
        if(x == 1)                       
        {
            uwb_flag = 1;
        }
        if(x == 2)
        {
            camera_flag = 1;
        }
        cout<<uwb_flag<<","<<camera_flag<<endl;
        if(uwb_flag==1&&camera_flag==1) //保证同步
        {
        double x1 = uwbe_pos.pose.pose.position.x ;
        double y1 = uwbe_pos.pose.pose.position.y;
        double x2 = vision_odom.pose.pose.position.x;
        double y2 = vision_odom.pose.pose.position.y;

         //赋值卡尔曼滤波输入
        X << x1,y1;
        Z << x2,y2;
        U << x1 - X_p(0,0),y1-X_p(1,0);
        //卡尔曼一次计算
        cout<<"X0:"<<X<<endl;
        X = A*X_p + B*U;
        cout<<"X1:"<<X<<endl;
        P = A*P_p*A.transpose()+Q;
        k = P*H.transpose()*((H*P*H.transpose()+R).inverse());
        X = X + k*(Z - H*X);
        cout<<"X2:"<<X<<endl;
        P = (I - k*H)*P;

        //记录
        uwb_log<<setprecision(12)<<x1<<","<<setprecision(12)<<y1<<","<<setprecision(12)<<uwbe_pos.pose.pose.orientation.w<<","<<setprecision(12)<<uwbe_pos.pose.pose.orientation.z<<endl;
        vision_log<<setprecision(12)<<x2<<","<<setprecision(12)<<y2<<endl;
        kalman_log<<setprecision(12)<<X(0,0)<<","<<setprecision(12)<<X(1,0)<<endl;
        X_p = X;
        P_p = P;

        fusion_odom.pose.pose.position.x = X(0,0);
        fusion_odom.pose.pose.position.y = X(1,0);
        fusion_odom.pose.pose.orientation.w = uwbe_pos.pose.pose.orientation.w;
        fusion_odom.pose.pose.orientation.z = uwbe_pos.pose.pose.orientation.z;
        pub_fusion.publish(fusion_odom);

        cout<<"uwb_x:"<<uwbe_pos.pose.pose.position.x <<"uwb_y:"<<uwbe_pos.pose.pose.position.y<<endl;
        cout<<"vision_x:"<<x2<<"vision_y:"<<y2<<endl;
        cout<<" kalman_x: "<<X(0,0)<<"kalman_y:"<<X(1,0)<<endl;
        cout<<"Q:"<<Q<<endl;
        cout<<"H:"<<H<<endl;
        cout<<"p:"<<P<<endl;
        cout<<"k:"<<k<<endl;
        cout<<"ok"<<endl<<endl;
        uwb_flag = 0;
        camera_flag = 0;
        }
    }
    void onmsg_odom(const nav_msgs::Odometry& msg)
    {
        odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
        odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
        odom.pose.pose.position.x =  msg.pose.pose.position.x ;
        odom.pose.pose.position.y =  msg.pose.pose.position.y ;
    }
    void onmsg_uwb_odom(const nav_msgs::Odometry& msg)
    {
        
        
        uwbe_pos.pose.pose.orientation.w = msg.pose.pose.orientation.w;
        uwbe_pos.pose.pose.orientation.z = msg.pose.pose.orientation.z;
        double r =atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z),1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z);
        uwbe_pos.pose.pose.position.x =  msg.pose.pose.position.x - delta*cos(r) ;
        uwbe_pos.pose.pose.position.y =  msg.pose.pose.position.y - delta*sin(r);
        // cout<<"r= "<<r<<endl;
        // cout<<"msgx: "<< msg.pose.pose.position.x<<"msg.y"<<msg.pose.pose.position.y <<endl;
        // cout<<"uwb_x:"<<uwbe_pos.pose.pose.position.x <<"uwb_y:"<<uwbe_pos.pose.pose.position.y<<endl;
        pub_uwbe.publish(uwbe_pos);
        int temp = 1;
        kalman_filter(temp);
    }        
    void onmsg_vision(const nav_msgs::Odometry& msg)
    {
	    // cout<<"get vision"<<endl;
        
        vision_odom.pose.pose.position.x = msg.pose.pose.position.x;
        vision_odom.pose.pose.position.y = msg.pose.pose.position.y;
        vision_odom.pose.pose.orientation.w = odom.pose.pose.orientation.w;
        vision_odom.pose.pose.orientation.z = odom.pose.pose.orientation.z;
        pub_vision.publish(vision_odom);
	    // cout<<"pub vison"<<endl;
        int temp = 2;
        kalman_filter(temp);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_uwb_odom;
    ros::Subscriber sub_vision;

    ros::Publisher pub_uwbe;
    ros::Publisher  pub_vision;
    ros::Publisher pub_fusion;

    nav_msgs::Odometry odom;
    nav_msgs::Odometry uwbe_pos;
    nav_msgs::Odometry vision_odom;
    nav_msgs::Odometry fusion_odom;

    //kalman 矩阵
    Matrix<double,2,2> A;
    Matrix<double,2,2> B;
    Matrix<double,2,1> U;
    Matrix<double,2,2> H;
    Matrix<double,2,2> R;
    Matrix<double,2,2> Q;
    Matrix<double,2,2> P;
    Matrix<double,2,2> P_p;
    Matrix<double,2,1> X;
    Matrix<double,2,1> X_p;
    Matrix<double,2,1> Z;
    Matrix<double,2,2> I;
    Matrix<double,2,2> k;

    //记数、融合
    bool uwb_flag;
    bool camera_flag;
};
int main(int argc,char ** argv)
{
    //记录数据
    uwb_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y,"<<setprecision(12)<<"w,"<<setprecision(12)<<"z"<<endl;
    vision_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;
    kalman_log<<setprecision(12)<<"x,"<<setprecision(12)<<"y"<<endl;

    ros::init(argc,argv,"sensor");
    sensorLayer sensor;
    ros::spin();
}
