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

//uwb视觉融合，单机器人控制
ofstream rb1_pos_log("rb1_pos_log_task3.csv",ios::out);
ofstream rb1_target_log("rb1_target_log_task3.csv",ios::out); //记录预设的目标点
double frequency_hz = 10;
class task
{
public:
    task()
    {
        pub_rb1_task = nh.advertise<geometry_msgs::Vector3>("target",1000);
        sub_rb1_uwb_odom = nh.subscribe("odom",1000,&task::onmsg_rb1_uwb_odom,this);
        rb1_total_num = 5000;
        rb1_num = 0;

        //做轨迹圆
        for(int i=0;i<rb1_total_num;i++)
        {
            double theta = 2*M_PI*i/5000.0;
            rb1_task[i*3] = 2*cos(theta-M_PI_2);
            rb1_task[i*3+1] = 2*sin(theta-M_PI_2) + 2;

            if(theta>M_PI) theta = theta - 2*M_PI;
            rb1_task[i*3+2] = theta;
            cout<<"x: "<<rb1_task[i*3]<<" y: "<<rb1_task[i*3+1]<<" r: "<<rb1_task[i*3+2]<<endl;
            rb1_target_log<<setprecision(12)<<rb1_task[i*3]<<","<<setprecision(12)<<rb1_task[i*3+1]<<",";
            rb1_target_log<<setprecision(12)<<rb1_task[i*3+2]<<endl;
        }
    }
    int nearest_find(double x,double y)
    {
	    double temp_dis;
	    double min_dis=100000;
	    int    res_num;
        double right_num = rb1_num + 500;//设置右查找边界
        if(rb1_num+500>=rb1_total_num) right_num = rb1_total_num -1;
	    for(int i=rb1_num;i<=right_num;i++)
	    {
		    temp_dis=sqrt((rb1_task[i*3]-x)*(rb1_task[i*3]-x)+(rb1_task[i*3+1]-y)*(rb1_task[i*3+1]-y));
		    if(temp_dis<=min_dis)
		    {
			    res_num=i;
			    min_dis=temp_dis;
		    }
	    }
	    return res_num;
    }

    void onmsg_rb1_uwb_odom(const nav_msgs::Odometry& msg)//接收rb1位置信息并更新rb1目标点
    {
        rb1_x = rb1_uwb_odom.pose.pose.position.x = msg.pose.pose.position.x;           //机器人1位置横坐标x
        rb1_y = rb1_uwb_odom.pose.pose.position.y = msg.pose.pose.position.y;           //机器人1位置横纵坐标y
        rb1_uwb_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;     //机器人1方向，以x轴为0，逆时针为正，范围(-1,1]
        double dis = sqrt((rb1_task[rb1_num*3]-rb1_x)*(rb1_task[rb1_num*3]-rb1_x)
                        +(rb1_task[rb1_num*3+1]-rb1_y)*(rb1_task[rb1_num*3+1]-rb1_y)); //机器人1当前位置到目标点距离
        cout<<"dis="<<dis<<endl;
        rb1_num = nearest_find(rb1_x,rb1_y);

        rb1_pos_log<<setprecision(12)<<rb1_x<<","<<setprecision(12)<<rb1_y<<endl;
    }
    
    void pub_task()
    {
        int rb1_nump = rb1_num;
        // if(rb1_nump >=rb1_total_num)
        // {
        //     rb1_nump = rb1_total_num-1;
        // }
        geometry_msgs::Vector3 tmp;
        tmp.x = rb1_task[rb1_nump*3];
        tmp.y = rb1_task[rb1_nump*3+1];
        tmp.z = rb1_task[rb1_nump*3+2];
        pub_rb1_task.publish(tmp);
        cout<<"rb1-task"<<tmp<<endl;
        cout<<"task num = "<<rb1_num<<endl;

    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_rb1_task;
    ros::Subscriber sub_rb1_uwb_odom;
    nav_msgs::Odometry rb1_uwb_odom;
    double rb1_task[5000*3];//num*3,保存目标点信息
    int rb1_num;
    int rb1_total_num;
    double rb1_x;
    double rb1_y;

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
