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


ofstream pos_log("pos_log.csv",ios::out);
ofstream target_log("target_log.csv",ios::out); //记录预设的目标点
double frequency_hz = 10;
class task
{
public:
    task()
    {
        pub_task = nh.advertise<geometry_msgs::Vector3>("target",1000);
        sub_uwb_odom = nh.subscribe("uwbe_odom",1000,&task::onmsg_uwb_odom,this);
        sub_camera_odom = nh.subscribe("vision_odom",1000,&task::onmsg_uwb_odom,this);
        sub_fusion_odom = nh.subscribe("fusion_odom",1000,&task::onmsg_uwb_odom,this);
        total_num = 5000;
        num = 0;
    
        //做轨迹圆
        for(int i=0;i<total_num;i++)
        {
            double theta = 2*M_PI*i/5000.0;
            target_point[i*3] = 0.75*cos(theta-M_PI_2) + 3;
            target_point[i*3+1] = 0.75*sin(theta-M_PI_2) + 2.5;

            if(theta>M_PI) theta = theta - 2*M_PI;
            target_point[i*3+2] = theta;
            cout<<"x: "<<target_point[i*3]<<" y: "<<target_point[i*3+1]<<" r: "<<target_point[i*3+2]<<endl;
            rb1_target_log<<setprecision(12)<<target_point[i*3]<<","<<setprecision(12)<<target_point[i*3+1]<<",";
            rb1_target_log<<setprecision(12)<<target_point[i*3+2]<<endl;
        }
        uwb_num = 0;
        camera_num =0;
    }
    int nearest_find(double x,double y)
    {
	    double temp_dis;
	    double min_dis=100000;
	    int    res_num;
        double right_num = num + 500;//设置右查找边界
        if(num+500>=total_num) right_num = total_num -1;
	    for(int i=num;i<=right_num;i++)
	    {
		    temp_dis=sqrt((target_point[i*3]-x)*(target_point[i*3]-x)+(target_point[i*3+1]-y)*(target_point[i*3+1]-y));
		    if(temp_dis<=min_dis)
		    {
			    res_num=i;
			    min_dis=temp_dis;
		    }
	    }
	    return res_num;
    }

    void onmsg_uwb_odom(const nav_msgs::Odometry& msg)//接收rb1位置信息并更新rb1目标点
    {
        uwb_num++;
        if(uwb_num - camera_num >10)
        {
            double x1;//机器人位置
            double y1;
            x1 = msg.pose.pose.position.x;           //机器人1位置横坐标x
            y1 =  msg.pose.pose.position.y;           //机器人1位置横纵坐标y
            double dis = sqrt((target_point[num*3]-x1)*(target_point[num*3]-x1)
                        +(target_point[num*3+1]-y1)*(target_point[num*3+1]-y1)); //机器人1当前位置到目标点距离
            cout<<"dis="<<dis<<endl;
            num = nearest_find(x1,y1);
        }
    }
    void onmsg_camera_odom(const nav_msgs::Odometry& msg)
    {
        camera_num ++;
        if(camera_num - uwb_num>10)
        {
            double x1;//机器人位置
            double y1;
            x1 = msg.pose.pose.position.x;           //机器人1位置横坐标x
            y1 =  msg.pose.pose.position.y;           //机器人1位置横纵坐标y
            double dis = sqrt((target_point[num*3]-x1)*(target_point[num*3]-x1)
                        +(target_point[num*3+1]-y1)*(target_point[num*3+1]-y1)); //机器人1当前位置到目标点距离
            cout<<"dis="<<dis<<endl;
            num = nearest_find(x1,y1);
        }
    }
    void onmsg_fusion_odom(const nav_msgs::Odometry& msg)
    {
            uwb_num = camera_num;
            double x1;//机器人位置
            double y1;
            x1 = msg.pose.pose.position.x;           //机器人1位置横坐标x
            y1 =  msg.pose.pose.position.y;           //机器人1位置横纵坐标y
            double dis = sqrt((target_point[num*3]-x1)*(target_point[num*3]-x1)
                        +(target_point[num*3+1]-y1)*(target_point[num*3+1]-y1)); //机器人1当前位置到目标点距离
            cout<<"dis="<<dis<<endl;
            num = nearest_find(x1,y1);
    }
    void pub_task()
    {
        int nump = num;
        // if(nump >=total_num)
        // {
        //     nump = total_num-1;
        // }
        geometry_msgs::Vector3 tmp;
        tmp.x = target_point[nump*3];
        tmp.y = target_point[nump*3+1];
        tmp.z = target_point[nump*3+2];
        pub_task.publish(tmp);
        cout<<"task"<<tmp<<endl;
        cout<<"task num = "<<num<<endl;

    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_task;
    ros::Subscriber sub_uwb_odom;
    nav_msgs::Odometry uwb_odom;
    double target_point[5000*3];//num*3,保存目标点信息
    int num;
    int total_num;


    int uwb_num,camera_num;

};
int main(int argc,char **argv)
{
    rb1_pos_log<<"x,"<<"y"<<endl;
    rb1_target_log<<"x,"<<"y,"<<"r"<<endl;
    ros::init(argc,argv,"task");
    task task1;
    ros::Rate loop_rate(frequency_hz);


    while(ros::ok())
    {
        task1.pub_task();//发布目标点；
        ros::spinOnce();
        loop_rate.sleep();
    }
}
