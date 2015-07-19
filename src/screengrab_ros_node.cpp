// copyright Lucas Walter November 2013

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <screengrab_ros/ScreenGrabConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// X Server includes
#include <X11/Xlib.h>
#include <X11/Xutil.h>

void XImage2RosImage(XImage& ximage, Display& _xDisplay, Screen& _xScreen,
    sensor_msgs::ImagePtr& im) 
{
    XColor color;
    
    im->header.stamp = ros::Time::now();

    if (_xScreen.depths->depth == 24) {
       // the code just deleted here is probably more robust than
       // a straight memcpy, but for now go without it.
       const int wd = ximage.width;
       const int ht = ximage.height;
       const int frame_size = wd * ht * 4;
       im->width = wd;
       im->height = ht;
       im->step = im->width * 4;
       // maybe this could be extracted from X
       im->encoding = sensor_msgs::image_encodings::BGRA8;
       im->data.resize(frame_size);
       memcpy(&im->data[0], ximage.data, frame_size); 

    } else { // Extremly slow alternative for non 24bit-depth
        Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
        for (unsigned int x = 0; x < ximage.width; x++) {
            for (unsigned int y = 0; y < ximage.height; y++) {
                color.pixel = XGetPixel(&ximage, x, y);
                XQueryColor(&_xDisplay, colmap, &color);
                //cv::Vec4b col = cv::Vec4b(color.blue, color.green, color.red,0);
                //tmp.at<cv::Vec4b> (y,x) = col;
            }
        }
    }
    return;
}

class ScreenGrab
{
  ros::NodeHandle nh_;
  
  ros::Publisher screen_pub_;
  
  int update_rate_;

  typedef dynamic_reconfigure::Server<screengrab_ros::ScreenGrabConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(screengrab_ros::ScreenGrabConfig &config,
      uint32_t level);

  void checkRoi(int& x_offset, int& y_offset, int& width, int& height);

  int x_offset_;
  int y_offset_;
  int width_;
  int height_;
  
  int screen_w_;
  int screen_h_;
  
  ros::Rate loop_rate_;

  boost::recursive_mutex dr_mutex_;

public:

  ScreenGrab();
  
  bool spin();

};
  
ScreenGrab::ScreenGrab() :
    x_offset_(0),
    y_offset_(0),
    width_(640),
    height_(480),
    loop_rate_(15)
    //server_(dr_mutex_) // this locks up
{
  screen_pub_ = nh_.advertise<sensor_msgs::Image>(
      "screengrab", 5);
  ros::param::param<int>("update_rate", update_rate_, 15);

  server_.reset(new ReconfigureServer(dr_mutex_)); 

  dynamic_reconfigure::Server<screengrab_ros::ScreenGrabConfig>::CallbackType cbt =
      boost::bind(&ScreenGrab::callback, this, _1, _2);
  server_->setCallback(cbt);
}

void ScreenGrab::checkRoi(int& x_offset, int& y_offset, int& width, int& height)
{
  // TODO with cv::Rect this could be one line rect1 & rect2
  
  // Need to check against resolution
  if ((x_offset + width) > screen_w_) 
  {
    // TBD need to more intelligently cap these
    if (screen_w_ > width) 
    {
      x_offset = screen_w_ - width;
    } 
    else 
    {
      x_offset = 0;
      width = screen_w_;
    }
  }

  if ((y_offset + height) > screen_h_) 
  {
    // TBD need to more intelligently cap these
    if (screen_h_ > height) 
    {
      y_offset = screen_h_ - height;
    } 
    else 
    {
      y_offset = 0;
      height = screen_h_;
    }
  }

}

void ScreenGrab::callback(
    screengrab_ros::ScreenGrabConfig &config,
    uint32_t level)
{
  if (level & 1)
  {
    checkRoi(config.x_offset, config.y_offset, config.width, config.height);
    x_offset_ = config.x_offset;
    y_offset_ = config.y_offset;
    width_ = config.width;
    height_ = config.height;
  }
  
  if (level & 2)
  {
    if (config.update_rate != update_rate_) 
    {
      loop_rate_ = ros::Rate(config.update_rate);
      update_rate_ = config.update_rate;
    }
  }
}

bool ScreenGrab::spin()
{
  // X resources
  Display* display;
  Screen* screen;
  XImage* xImageSample;
  XColor col;

  // init
  // from vimjay screencap.cpp (https://github.com/lucasw/vimjay)
  {
  display = XOpenDisplay(NULL); // Open first (-best) display
  if (display == NULL) {
    ROS_ERROR_STREAM("bad display");
    return false;
  }

  screen = DefaultScreenOfDisplay(display);
  if (screen == NULL) {
    ROS_ERROR_STREAM("bad screen");
    return false;
  }

  Window wid = DefaultRootWindow( display );
  if ( 0 > wid ) {
    ROS_ERROR_STREAM("Failed to obtain the root windows Id "
        "of the default screen of given display.\n");
    return false;
  }

  XWindowAttributes xwAttr;
  Status ret = XGetWindowAttributes( display, wid, &xwAttr );
  screen_w_ = xwAttr.width;
  screen_h_ = xwAttr.height;
  }

  // get initial values from parameter server, override
  // dr cfg defaults
  ros::param::param<int>("update_rate", update_rate_, 15);
  ros::param::param<int>("x_offset", x_offset_, 0);
  ros::param::param<int>("y_offset", y_offset_, 0);
  ros::param::param<int>("width", width_, 640);
  ros::param::param<int>("height", height_, 480);
  checkRoi(x_offset_, y_offset_, width_, height_);

  screengrab_ros::ScreenGrabConfig config;
  config.update_rate = update_rate_;
  config.x_offset = x_offset_;
  config.y_offset = y_offset_;
  config.width = width_;
  config.height = height_;

  //boost::recursive_mutex::scoped_lock lock(dr_mutex_);
  server_->updateConfig(config);
  //lock.unlock();

  while (ros::ok()) 
  {
    sensor_msgs::ImagePtr im(new sensor_msgs::Image);
    
    // grab the image
    {
      xImageSample = XGetImage(display, DefaultRootWindow(display),
          x_offset_, y_offset_, width_, height_, AllPlanes, ZPixmap);

      // Check for bad null pointers
      if (xImageSample == NULL) {
        ROS_ERROR_STREAM("Error taking screenshot! "
            << ", " << x_offset_ << " " << y_offset_ 
            << ", " << width_ << " " << height_
            << ", " << screen_w_ << " " << screen_h_
            );
        continue;
      }
      
      // convert to Image format
      XImage2RosImage(*xImageSample, *display, *screen, im);
      
      XDestroyImage(xImageSample);

    }
    
    screen_pub_.publish(im);

    ros::spinOnce();
    loop_rate_.sleep();
  }

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "screengrab_ros_node");

  ScreenGrab screen_grab;
  screen_grab.spin();
}
