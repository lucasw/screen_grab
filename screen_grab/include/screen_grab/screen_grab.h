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

#ifndef SCREEN_GRAB_SCREEN_GRAB_H
#define SCREEN_GRAB_SCREEN_GRAB_H

#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <screen_grab/ScreenGrabConfig.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <string>
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

  std::string encoding_ = "bgra8";

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
