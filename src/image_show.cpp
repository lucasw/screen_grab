/**

  Nodelet based image display.

  Use cv::imshow for now, later look at what I was using in vimjay earlier 
  (something where I could make the window undecorated, control the width and height?)

  Turn this into a standalone undecorated image viewer
*/

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace image_show
{

class ImageShow : public nodelet::Nodelet
{
  image_transport::ImageTransport* it_; 
  
  image_transport::Subscriber image_sub_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  ros::Timer timer_;

public:
  virtual void onInit();
 
  ImageShow();
};

}


#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_show::ImageShow, nodelet::Nodelet)

namespace image_show
{

ImageShow::ImageShow() 
{

}

void ImageShow::onInit()
{
  it_ = new image_transport::ImageTransport(getNodeHandle());
  
  image_sub_ = it_->subscribe("image", 1, &ImageShow::imageCallback, this);
}

void ImageShow::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::imshow("image", cv_ptr->image);
  cv::waitKey(5); 
}

}
