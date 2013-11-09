#include "ros/ros.h"
#include "sensor_msgs/Image.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "screengrab_ros_node");

  ros::NodeHandle nh;

  ros::Publisher screen_pub = nh.advertise<sensor_msgs::Image>(
      "screengrab", 5);
  // TBD make teh rate a ros param
  ros::Rate loop_rate(10);

  while (ros::ok()) 
  {
    sensor_msgs::ImagePtr im; //(new sensor_msgs::Image);
    
    // grab the image
    // convert to Image format
    // screen_pub.publish(im);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
