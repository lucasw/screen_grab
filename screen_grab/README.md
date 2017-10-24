screen_grab
===========

Capture the screen and publish it on a ROS `sensor_msgs/Image` topic

Currently this runs as a nodelet.
`screengrab.launch` shows an example launch file, it will probably need to be copied and altered for integration in a real system.

    roslaunch screen_grab screengrab.launch

A nodelet image display window will also pop up showing the captured image of the screen, and of course `rqt_image_view` or any image subscriber can be used.

The built package is available for jade:

    sudo apt-get install ros-jade-screen-grab

But building for hydro and indigo ought to work.

A video of an earlier version: https://www.youtube.com/watch?v=Ys9BN4mJ_yc

Some discussion is here: https://plus.google.com/+LucasWalter/posts/f7fLhyWL3A4

