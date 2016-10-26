#ifndef SCREEN_GRAB_SCREEN_GRAB_H
#define SCREEN_GRAB_SCREEN_GRAB_H

#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <screen_grab/ScreenGrabConfig.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <X11/Xlib.h>

namespace screen_grab
{

class ScreenGrab : public nodelet::Nodelet
{
  ros::Publisher screen_pub_;

  ros::Subscriber roi_sub_;
  void roiCallback(const sensor_msgs::RegionOfInterest::ConstPtr& msg);

  double update_rate_;

  typedef dynamic_reconfigure::Server<screen_grab::ScreenGrabConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(screen_grab::ScreenGrabConfig &config,
                uint32_t level);

  void checkRoi(int& x_offset, int& y_offset, int& width, int& height);
  void updateConfig();

  int x_offset_;
  int y_offset_;
  int width_;
  int height_;

  int screen_w_;
  int screen_h_;

  boost::recursive_mutex dr_mutex_;

  void spinOnce(const ros::TimerEvent& e);
  bool first_error_;

  ros::Timer timer_;

  // X resources
  Display* display;
  Screen* screen;
  XImage* xImageSample;
  XColor col;

public:
  virtual void onInit();

  ScreenGrab();

  bool spin();
};

}  //  namespace screen_grab

#endif  // SCREEN_GRAB_SCREEN_GRAB_H
