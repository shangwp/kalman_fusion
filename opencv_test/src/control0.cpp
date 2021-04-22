#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
using namespace std;
class controlLayer
{
public:
    controlLayer()
    {
            pub_move = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1000); 
    }
    void pub_control()
    {
        move_cmd.linear.x =0.2;
        move_cmd.angular.z = 0;
         pub_move.publish(move_cmd);

    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_move;
    geometry_msgs::Twist move_cmd;//控制指令
};

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"control");
    controlLayer control;
    double frequency_hz = 10;
    ros::Rate loop_rate(frequency_hz);
    
    while(ros::ok())
    {
        control.pub_control();//发布目标点；
        ros::spinOnce();
        loop_rate.sleep();
    }
}
