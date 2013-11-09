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
        // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
        unsigned long rmask = _xScreen.root_visual->red_mask,
                gmask = _xScreen.root_visual->green_mask,
                bmask = _xScreen.root_visual->blue_mask;
        unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
        unsigned char colorChannel[3];

        rshift = 0;
        rbits = 0;
        while (!(rmask & 1)) {
            rshift++;
            rmask >>= 1;
        }
        while (rmask & 1) {
            rbits++;
            rmask >>= 1;
        }
        if (rbits > 8) {
            rshift += rbits - 8;
            rbits = 8;
        }

        gshift = 0;
        gbits = 0;
        while (!(gmask & 1)) {
            gshift++;
            gmask >>= 1;
        }
        while (gmask & 1) {
            gbits++;
            gmask >>= 1;
        }
        if (gbits > 8) {
            gshift += gbits - 8;
            gbits = 8;
        }

        bshift = 0;
        bbits = 0;
        while (!(bmask & 1)) {
            bshift++;
            bmask >>= 1;
        }
        while (bmask & 1) {
            bbits++;
            bmask >>= 1;
        }
        if (bbits > 8) {
            bshift += bbits - 8;
            bbits = 8;
        }

       const int wd = ximage.width;
       const int ht = ximage.height;
       const int frame_size = wd * ht * 4;
       im->width = wd;
       im->height = ht;
       im->step = im->width * 3;
       im->encoding = sensor_msgs::image_encodings::BGRA8;
       im->data.resize(frame_size);
       memcpy(&im->data[0], ximage.data, frame_size); 
       #if 0
        for (unsigned int x = 0; x < ximage.width; x++) {
            for (unsigned int y = 0; y < ximage.height; y++) {
                color.pixel = XGetPixel(&ximage, x, y);
                colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
                colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
                colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
                cv::Vec4b col = cv::Vec4b(colorChannel[0], colorChannel[1], colorChannel[0],0);
                tmp.at<cv::Vec4b> (y,x) = col;
            }
        }
        #endif
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "screengrab_ros_node");

  ros::NodeHandle nh;

  ros::Publisher screen_pub = nh.advertise<sensor_msgs::Image>(
      "screengrab", 5);
  // TBD make teh rate a ros param
  ros::Rate loop_rate(10);

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
    return -1;
  }

  screen = DefaultScreenOfDisplay(display);
  if (screen == NULL) {
    ROS_ERROR_STREAM("bad screen");
    return -2;
  }

  Window wid = DefaultRootWindow( display );
  if ( 0 > wid ) {
    ROS_ERROR_STREAM("Failed to obtain the root windows Id "
        "of the default screen of given display.\n");
    return -3;
  }

  XWindowAttributes xwAttr;
  Status ret = XGetWindowAttributes( display, wid, &xwAttr );
  screen_w = xwAttr.width;
  screen_h = xwAttr.height;
  }


  while (ros::ok()) 
  {
    sensor_msgs::ImagePtr im(new sensor_msgs::Image);
    
    // grab the image
    {
      int startX = 0;
      int startY = 0;
      int widthX = 640;
      int heightY = 480;

      // Need to check against resolution
      if ((startX + widthX) > screen_w) {
        // TBD need to more intelligently cap these
        if (screen_w > widthX) {
          startX = screen_w - widthX;
        } else {
          startX = 0;
          widthX = screen_w;
        }
      }

      if ((startY + heightY) > screen_h) {
        // TBD need to more intelligently cap these
        if (screen_h > heightY) {
          startY = screen_h - heightY;
        } else {
          startY = 0;
          heightY = screen_h;
        }
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
    
    screen_pub.publish(im);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
