#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;
ofstream control_log("control_log.csv",ios::out);
//用于仿真圆循迹
/*
输入： 里程计位置
      下一个目标点
输出： 控制量
*/
//根据里程计信息控制行动，以及简单测试是否能运动
class controlLayer
{
public:
    controlLayer()
    {
	    cout<<"init"<<endl;
        sub_target = nh.subscribe("target",1000,&controlLayer::onmsg_target,this);
        sub_uwb_odom = nh.subscribe("uwbe_odom",1000,&controlLayer::onmsg_uwb_odom,this);
        sub_camera_odom = nh.subscribe("vision_odom",1000,&controlLayer::onmsg_camera_odom,this);
        sub_fusion_odom = nh.subscribe("fusion_odom",1000,&controlLayer::onmsg_fusion_odom,this);
        pub_move = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1000); 
        flag = 0;//待接收到目标点后才开始运动
        //stdev = 0.0;//标准差
        //num = 0;
        max_error = 0;
        max_error_2 = 0;
        uwb_num = 0;
        camera_num =0;
    }
// void stanley_method()
// {  
// }

//接收目标点，保存到target
void onmsg_target(const geometry_msgs::Vector3& msg)
{
    target.x = msg.x;
    target.y = msg.y;
    target.z = msg.z;
    flag = 1;
}
double std_deviation(double a, double b,double c,double d)
{
    return sqrt((a-b)*(a-b)/2+(c-d)*(c-d)/2);
}
void stanley_method(double x,double y,double r,double dis)
{
	flag = 1;
    control_log<<setprecision(12)<<target.x<<","<<setprecision(12)<<target.y<<","<<setprecision(12)<<target.z<<",";
    cout<<"x= "<<x<<"y= "<<y<<"r="<<r<<endl;
    control_log<<setprecision(12)<<x<<","<<setprecision(12)<<y<<","<<setprecision(12)<<r<<",";
    cout<<"dis= "<<dis<<endl;
    control_log<<setprecision(12)<<dis;

        //注意，turtlebot中r为-M_PI，M_PI，朝向以x轴朝向为0，逆时针正，顺时针负
        double k = 2;
        double v = 0.2;
        //坐标系，朝向以x轴朝向为0，逆时针正，顺时针负
        double delta_r = target.z - r;// 1.方向差
        if(delta_r>M_PI) delta_r = delta_r - 2*M_PI;
        else if(delta_r<=-M_PI) delta_r = delta_r + 2*M_PI;
        cout<<"delta_r= "<<delta_r<<endl;
        control_log<<","<<setprecision(12)<<delta_r<<",";
        double e_dis;//横向距离
        double degree_r2p;
        degree_r2p = atan((target.y-y)/(target.x-x));
        if(target.x<=x&&target.y<y) degree_r2p = degree_r2p - M_PI;
        else if(target.x<=x&&target.y>=y) degree_r2p = degree_r2p + M_PI;

        double delta_degree_e = target.z - degree_r2p;
        if(delta_degree_e>M_PI) delta_degree_e = delta_degree_e - 2*M_PI;
        else if(delta_degree_e<=-M_PI) delta_degree_e = delta_degree_e + 2*M_PI;
        e_dis = fabs(dis*sin(delta_degree_e*M_PI));
        double et = atan(k*e_dis/v);

        if(delta_degree_e>=0)// 根据无人船朝向和船到目标点的方位以及目标点朝向确定et对应的转向。
        {
            if(fabs(delta_r)<M_PI_2) et = -et;
            else et = et;
        }
        else if(delta_degree_e<0)
        {
            if(fabs(delta_r)<M_PI_2) et = et;
            else et = -et;
        }


        cout<<"degree_r2p= "<<degree_r2p<<"delta_degree_e= "<<delta_degree_e;
        control_log<<setprecision(12)<<degree_r2p<<","<<setprecision(12)<<delta_degree_e<<",";
        cout<<"e_dis= "<<e_dis<<"et= "<<et<<endl;
        control_log<<setprecision(12)<<e_dis<<","<<setprecision(12)<<et<<",";
        double delta_sum = et + delta_r;
        if(delta_sum>M_PI) delta_sum = delta_sum - 2*M_PI;
        else if(delta_sum<=-M_PI) delta_sum = delta_sum + 2*M_PI;
        cout<<"delta_sum= "<<delta_sum;
        control_log<<setprecision(12)<<delta_sum<<",";
        move_cmd.linear.x = 0.2;
        move_cmd.angular.z = delta_sum/M_PI*2.5;
        if(move_cmd.angular.z>1) move_cmd.angular.z = 1;
        else if(move_cmd.angular.z<-1) move_cmd.angular.z = -1;

        control_log<<setprecision(12)<<move_cmd.linear.x <<","<<setprecision(12)<<move_cmd.angular.z<<endl;
        if(flag==1)  pub_move.publish(move_cmd);
        cout<<move_cmd<<endl;
}


    void onmsg_uwb_odom(const nav_msgs::Odometry& msg)//接收rb1位置信息并更新rb1目标点
    {
        uwb_num++;
	cout<<uwb_num<<endl;
        if(uwb_num - camera_num >10)
        {
            double x = msg.pose.pose.position.x;     //机器人位置横坐标x
            double y = msg.pose.pose.position.y;     //机器人位置纵坐标y
            //机器人方向，以x轴为0，逆时针为正，范围(-1,1]
            double r = atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z),1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z);
            double dis = sqrt((target.x-x)*(target.x-x)+(target.y-y)*(target.y-y)); //机器人当前位置到目标点距离
	    stanley_method(x,y,r,dis);
        }
    }
    void onmsg_camera_odom(const nav_msgs::Odometry& msg)
    {
        camera_num ++;
	cout<<camera_num<<endl;
        if(camera_num - uwb_num>10)
        {
                double x = msg.pose.pose.position.x;     //机器人位置横坐标x
                 double y = msg.pose.pose.position.y;     //机器人位置纵坐标y
                //机器人方向，以x轴为0，逆时针为正，范围(-1,1]
                double r = atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z),1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z);
                double dis = sqrt((target.x-x)*(target.x-x)+(target.y-y)*(target.y-y)); //机器人当前位置到目标点距离
		stanley_method(x,y,r,dis);
        }
    }
    void onmsg_fusion_odom(const nav_msgs::Odometry& msg)
    {

        double x = msg.pose.pose.position.x;     //机器人位置横坐标x
        double y = msg.pose.pose.position.y;     //机器人位置纵坐标y
        //机器人方向，以x轴为0，逆时针为正，范围(-1,1]
        double r = atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z),1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z);
        double dis = sqrt((target.x-x)*(target.x-x)+(target.y-y)*(target.y-y)); //机器人当前位置到目标点距离
        stanley_method(x,y,r,dis);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub_move;
    ros::Subscriber sub_uwb_odom;
    ros::Subscriber sub_camera_odom;
    ros::Subscriber sub_fusion_odom;
    ros::Subscriber sub_target;
    geometry_msgs::Twist move_cmd;//控制指令
    geometry_msgs::Vector3 target;//目标点
    int flag;
    //long int num;//记录点数，并用于计算标准差
    //double stdev;
    double max_error;
    double max_error_2;
    int uwb_num,camera_num;
};

int main(int argc,char ** argv)
{
    control_log<<"tx,"<<"ty,"<<"tr,"<<"x,"<<"y,"<<"r,"<<"dis,"<<"delta_r,"<<"degree_r2p,"<<"delta_degree_e,"<<"e_dis,"<<"et,"<<"delta_sum,"<<"cmd_x,"<<"cmd_y"<<endl;
    ros::init(argc,argv,"control");
    controlLayer control;
    ros::spin();
}

// //接收odom信息，并根据当前位置和目标点发出控制指令
// void onmsg_odom(const nav_msgs::Odometry& msg)
// {
    
//     // move_cmd.linear.x = 0.2;
//     // move_cmd.angular.z = -0.5;
//     // pub_move.publish(move_cmd);
//     // cout<<"ok"<<endl;


//     // if(dis<M_PI/100)
//     // {
//     //     move_cmd.linear.x = 0;
//     //     move_cmd.angular.z = 0;
//     //     if(flag=1) pub_move.publish(move_cmd);
//     //     control_log<<endl;
//     // }
//     // else if(dis>=M_PI/100)
//     // {
//     stanley_method(x,y,r,dis);
//     // }

//     // num++;
//     // stdev = (num-1)*stdev/num + std_deviation(target.x,x,target.y,y)/num;
//     // cout<<"stdev: "<<stdev<<endl;
//     // if(std_deviation(target.x,x,target.y,y)>=max_error)
//     // {
//     //     max_error = std_deviation(target.x,x,target.y,y);
//     // }
//     // else if(std_deviation(target.x,x,target.y,y)>max_error_2)
//     // {
//     //     max_error_2 = std_deviation(target.x,x,target.y,y);
//     // }
//     // cout<<"max_error: "<<max_error<<endl;
//     // cout<<"max_error2: "<<max_error_2<<endl;
// }
