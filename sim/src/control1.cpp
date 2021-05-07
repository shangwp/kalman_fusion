#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
using namespace std;


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
        sub_target = nh.subscribe("target",1000,&controlLayer::onmsg_target,this);//rb1代表机器人编号1
        sub_odom = nh.subscribe("odom",1000,&controlLayer::onmsg_odom,this);
        pub_move = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1000); 
        flag = 0;//待接收到目标点后才开始运动
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


//接收odom信息，并根据当前位置和目标点发出控制指令
void onmsg_odom(const nav_msgs::Odometry& msg)
{
    
    // move_cmd.linear.x = 0.2;
    // move_cmd.angular.z = -0.5;
    // pub_move.publish(move_cmd);
    // cout<<"ok"<<endl;
    double x = msg.pose.pose.position.x;     //机器人位置横坐标x
    double y = msg.pose.pose.position.y;     //机器人位置纵坐标y
    double r = msg.pose.pose.orientation.z;  //机器人方向，以x轴为0，逆时针为正，范围(-1,1]
    cout<<"x= "<<x<<"y= "<<y<<"r="<<r<<endl;
    double dis = sqrt((target.x-x)*(target.x-x)+(target.y-y)*(target.y-y)); //机器人当前位置到目标点距离
    cout<<"dis= "<<dis<<endl;
    if(dis<)
    {
        move_cmd.linear.x = 0;
        move_cmd.angular.z = 0;
        if(flag=1) pub_move.publish(move_cmd);
    }
    else if(dis>=0.2)
    {
        //stanley_method()
        //坐标系，朝向以x轴朝向为0，逆时针正，顺时针负
        double delta_r = target.z - r;// 1.方向差
        if(delta_r>1) delta_r = delta_r - 2;
        else if(delta_r<=-1) delta_r = delta_r + 2;
        cout<<"delta_r= "<<delta_r<<endl;

        double e_dis;//横向距离
        double degree_r2p;
        degree_r2p = atan((target.y-y)/(target.x-x));
        if(target.x<=x&&target.y<y) degree_r2p = degree_r2p - M_PI;
        else if(target.x<=x&&target.y>=y) degree_r2p = degree_r2p + M_PI;
        degree_r2p = degree_r2p/M_PI;
        double delta_degree_e = target.z - degree_r2p;
        if(delta_degree_e>1) delta_degree_e = delta_degree_e - 2;
        else if(delta_degree_e<=-1) delta_degree_e = delta_degree_e + 2;
        e_dis = fabs(dis*sin(delta_degree_e*M_PI));
        double et = atan(0.2*e_dis*e_dis);
        if(delta_degree_e>=0) et = -et;
        cout<<"degree_r2p= "<<degree_r2p<<"delta_degree_e= "<<delta_degree_e;
        cout<<"e_dis= "<<e_dis<<"et= "<<et<<endl;

        double delta_sum = et + delta_r;
        if(delta_sum>1) delta_sum = delta_sum - 2;
        else if(delta_sum<=-1) delta_sum = delta_sum + 2;
        cout<<"delta_sum= "<<delta_sum;
        move_cmd.linear.x = 0.2;
        move_cmd.angular.z = 0.5*delta_sum;
        if(flag=1)  pub_move.publish(move_cmd);
        cout<<move_cmd<<endl;
    }

}

private:
    ros::NodeHandle nh;
    ros::Publisher pub_move;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_target;
    geometry_msgs::Twist move_cmd;//控制指令
    geometry_msgs::Vector3 target;//目标点
    int flag;
};

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"control");
    controlLayer control;
    ros::spin();
}
