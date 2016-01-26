^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package screen_grab
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2016-01-26)
------------------

0.0.1 (2016-01-22)
------------------
* Looks like opencv imshow can't be used from two nodelets in the same group.
* Fixed nodelet xml for image show, now it works
* Install launch files.  It looks like image_view has some problems 'Could not create object of class type image_show::ImageShow as no factory exists for it.'
* catkin_lint cleanup
* New image_show was useful for debugging screen_grab nodelet, may move elsewhere later, also is much less cpu than image_view or rqt_image_view.  ScreenGrab ndelet is now fully operational. `#5 <https://github.com/lucasw/screengrab_ros/issues/5>`_
* Looks like the setting/getting of parameters is working now (and maybe can be simplified further), but I haven't run image view as a nodelet yet.  I have discovered that connecting to the nodelet manager with a bad node name brings it to a halt (am I not printing output to screen for it?), so maybe my image view command isn't right.  If nodelets are that brittle then they become a lot less useful whatever the performance gains.
* Nodelet merged with roi code, also dr is using private namespace.
* Implemented nodelet `#5 <https://github.com/lucasw/screengrab_ros/issues/5>`_, but need to merge with RegionOfInterest changes
* `#4 <https://github.com/lucasw/screengrab_ros/issues/4>`_ RegionOfInterest control in parallel to dynamic reconfigure.
* Now dynamic reconfigure is working, though it would be nice to see if I can set slider maximums at start time and override the cfg.
* Now can set the update rate dynamically, there is no Rate::set method but it can just be overwritten with new Rate instance.
* Putting all parameters into a namespace, make sure everything uses relative namespace names
* Have a launch file that also launch image_view
* Using ros parameters to set x,y,width, & height, next add defaults to new launch file.
* Taking code from vimjay screencap to work here, this does no conversion yet.
* Contributors: Lucas Walter
