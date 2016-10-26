#include <nodelet/loader.h>
#include <ros/ros.h>
// #include <screen_grab/screen_grab.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "screen_grab");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "screen_grab/ScreenGrab", remap, nargv);
  ros::spin();
}
