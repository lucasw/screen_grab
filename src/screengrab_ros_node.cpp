// copyright Lucas Walter November 2013

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

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

public:

  ScreenGrab();
  
  bool spin();
};
  
ScreenGrab::ScreenGrab()
{

  screen_pub_ = nh_.advertise<sensor_msgs::Image>(
      "screengrab", 5);
  ros::param::param<int>("update_rate", update_rate_, 15);

}

bool ScreenGrab::spin()
{
  ros::Rate loop_rate(update_rate_);

  // X resources
  Display* display;
  Screen* screen;
  XImage* xImageSample;
  XColor col;

  int screen_w;
  int screen_h;

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
  screen_w = xwAttr.width;
  screen_h = xwAttr.height;
  }

  int startX = 0;
  int startY = 0;
  int widthX = 640;
  int heightY = 480;
  
  while (ros::ok()) 
  {
    sensor_msgs::ImagePtr im(new sensor_msgs::Image);
    
    int new_update_rate;
    ros::param::param<int>("update_rate", new_update_rate, 15);
    if (new_update_rate != update_rate_) {
      loop_rate = ros::Rate(new_update_rate);
      update_rate_ = new_update_rate;
    }
    // grab the image
    {
      ros::param::param<int>("start_x", startX, 0);
      ros::param::param<int>("start_y", startY, 0);
      ros::param::param<int>("width_x", widthX, 640);
      ros::param::param<int>("height_y", heightY, 480);

      // Need to check against resolution
      bool changed = false;
      if ((startX + widthX) > screen_w) {
        // TBD need to more intelligently cap these
        if (screen_w > widthX) {
          startX = screen_w - widthX;
        } else {
          startX = 0;
          widthX = screen_w;
          ros::param::set("width_x", widthX);
        }
        ros::param::set("start_x", startY);
        changed = true;
      }

      if ((startY + heightY) > screen_h) {
        // TBD need to more intelligently cap these
        if (screen_h > heightY) {
          startY = screen_h - heightY;
        } else {
          startY = 0;
          heightY = screen_h;
          ros::param::set("height_y", heightY);
        }
        ros::param::set("start_y", startY);
        changed = true;
      }
      
      if (changed) {
        ROS_WARN_STREAM("parameters had to change " << startX << " " 
            << startY << " " << widthX << " " << heightY);
      }

      xImageSample = XGetImage(display, DefaultRootWindow(display),
          startX, startY, widthX, heightY, AllPlanes, ZPixmap);

      // Check for bad null pointers
      if (xImageSample == NULL) {
        ROS_ERROR_STREAM("Error taking screenshot!");
        continue;
      }
      
      // convert to Image format
      XImage2RosImage(*xImageSample, *display, *screen, im);
      
      XDestroyImage(xImageSample);

    }
    
    screen_pub_.publish(im);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "screengrab_ros_node");

  ScreenGrab screen_grab = ScreenGrab();
  screen_grab.spin();
}
