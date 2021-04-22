#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
static const char WINDOW[] = "Image window";
static void help()
{
    printf("\nThis program demonstrates converting OpenCV Image to ROS Image messages  \n"
        );
 
}
 
int main(int argc, char** argv)
{
  help();
  ros::init(argc, argv, "image_converter");
 
  //Reading an image from the file
  cv::Mat cv_image = cv::imread("/home/shang/图片/1.png");
  if(cv_image.empty() )
    {
        ROS_ERROR("Read the picture failed!");
        return -1;
    }
 

  //Show the image
  cv::namedWindow(WINDOW);
  cv::imshow(WINDOW,cv_image);
  cv::waitKey(0);
 
  ros::spin();
  return 0;
}
