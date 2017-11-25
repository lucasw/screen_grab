/*
 * Copyright (c) 2013 Lucas Walter
 * November 2013
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rclcpp/clock.hpp>
#include <rclcpp/time_source.hpp>
#include <screen_grab/screen_grab.h>
#include <sensor_msgs/image_encodings.hpp>

// X Server includes
#include <X11/Xutil.h>
using std::placeholders::_1;

void XImage2RosImage(XImage& ximage, Display& _xDisplay, Screen& _xScreen,
                     sensor_msgs::msg::Image& im)
{
  XColor color;

  if (_xScreen.depths->depth == 24)
  {
    // the code just deleted here is probably more robust than
    // a straight memcpy, but for now go without it.
    const int wd = ximage.width;
    const int ht = ximage.height;
    const int frame_size = wd * ht * 4;
    im.width = wd;
    im.height = ht;
    im.step = im.width * 4;
    // maybe this could be extracted from X
    im.encoding = sensor_msgs::image_encodings::BGRA8;
    im.data.resize(frame_size);
    memcpy(&im.data[0], ximage.data, frame_size);
  }
  else     // Extremly slow alternative for non 24bit-depth
  {
    Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
    for (int x = 0; x < ximage.width; x++)
    {
      for (int y = 0; y < ximage.height; y++)
      {
        color.pixel = XGetPixel(&ximage, x, y);
        XQueryColor(&_xDisplay, colmap, &color);
        // cv::Vec4b col = cv::Vec4b(color.blue, color.green, color.red,0);
        // tmp.at<cv::Vec4b> (y,x) = col;
      }
    }
  }
  return;
}

ScreenGrab::ScreenGrab(rclcpp::node::Node::SharedPtr clock_node) :
  Node("screen_grab"),
  ts_(clock_node),
  x_offset_(0),
  y_offset_(0),
  width_(640),
  height_(480),
  update_rate_(10.0),
  first_error_(false)
{
  onInit();
}

void ScreenGrab::roiCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
{
  x_offset_ = msg->x_offset;
  y_offset_ = msg->y_offset;
  width_ = msg->width;
  height_ = msg->height;

  updateConfig();
}

void ScreenGrab::frameRateCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (msg->data <= 0.0)
  {
    RCLCPP_ERROR(this->get_name(), "bad update rate %f", msg->data);
    // TODO(lucasw) disable any existing timer_?
    return;
  }
  update_rate_ = msg->data;
  const float period = 1.0 / update_rate_;
  // TODO(lucasw) does RCLCPP_INFO log, or is RCUTILS_LOG_INFO_NAMED needed?
  // does it publish to a ros topic?
  RCLCPP_INFO(this->get_name(), "period %f, update rate %f", period, update_rate_);
  std::chrono::nanoseconds period_ns(static_cast<int>(period * 1e9));
  timer_ = this->create_wall_timer(
      period_ns, std::bind(&ScreenGrab::spinOnce, this));
}

void ScreenGrab::checkRoi(int& x_offset, int& y_offset, int& width, int& height)
{
  // TODO(lucasw) with cv::Rect this could be one line rect1 & rect2

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

  // RCLCPP_INFO(this->get_name(), x_offset << " " << y_offset << " " << width << " " << height);
}

void ScreenGrab::updateConfig()
{
  checkRoi(x_offset_, y_offset_, width_, height_);
}

void ScreenGrab::onInit()
{
  screen_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image");

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts_.attachClock(clock_);

  // TODO(lucasw) move most of this into onInit
  // init
  // from vimjay screencap.cpp (https://github.com/lucasw/vimjay)
  {
    display = XOpenDisplay(NULL);  // Open first (-best) display
    if (display == NULL)
    {
      RCLCPP_ERROR(get_name(), "bad display");
      return;
    }

    screen = DefaultScreenOfDisplay(display);
    if (screen == NULL)
    {
      RCLCPP_ERROR(get_name(), "bad screen");
      return;
    }

    Window wid = DefaultRootWindow(display);
    #if 0
    if (0 > wid)
    {
      RCLCPP_ERROR(get_name(), "Failed to obtain the root windows Id "
                       "of the default screen of given display.\n");
      return;
    }
    #endif

    XWindowAttributes xwAttr;
    // TODO(lucasw) check Status
    // Status ret =
    XGetWindowAttributes(display, wid, &xwAttr);
    screen_w_ = xwAttr.width;
    screen_h_ = xwAttr.height;
  }

  updateConfig();

  roi_sub_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
        "roi", std::bind(&ScreenGrab::roiCallback, this, _1));

  auto msg = std::make_shared<std_msgs::msg::Float64>();
  msg->data = update_rate_;
  frameRateCallback(msg);

  frame_rate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "frame_rate", std::bind(&ScreenGrab::frameRateCallback, this, _1));
}

void ScreenGrab::spinOnce()
{
  builtin_interfaces::msg::Time timestamp = clock_->now();
  // std::cout << timestamp.sec << " " << timestamp.nanosec << "\n";
  sensor_msgs::msg::Image im;

  // grab the image
  xImageSample = XGetImage(display, DefaultRootWindow(display),
                           x_offset_, y_offset_, width_, height_, AllPlanes, ZPixmap);

  // Check for bad null pointers
  if (xImageSample == NULL)
  {
    if (first_error_)
      RCLCPP_ERROR(get_name(), "Error taking screenshot! off %d %d, wh %d %d, screen %d %d",
                       x_offset_, y_offset_,
                       width_, height_,
                       screen_w_, screen_h_);
    first_error_ = false;
    return;
  }

  if (!first_error_)
    RCLCPP_INFO(this->get_name(), "%d %d", width_, height_);
  first_error_ = true;
  // convert to Image format
  im.header.stamp = timestamp;
  XImage2RosImage(*xImageSample, *display, *screen, im);

  XDestroyImage(xImageSample);

  screen_pub_->publish(im);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // TODO(lucasw) would like to construct a timesource within ScreenGrab but not clear on how to do it,
  // it requires a shared_ptr to the node.
  auto clock_node = rclcpp::node::Node::make_shared("clock_node");
  auto node = std::make_shared<ScreenGrab>(clock_node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
