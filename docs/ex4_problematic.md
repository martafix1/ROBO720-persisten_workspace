
visualise the frames in RVIZ
this can be done using the ros2 tf, some of them should be probs already existent in the gazebo. Base is likely "panda_link0".

Also the timestamp for the tf is probs wrong when using get_node()->get_clock() or something like that. However using the pandalink0 as a base should hopefuly propagate that and work around it. 

also invert the output from the aruko, its not from marker to camera, but from camera to marker.