# switch_finder

*switch_finder* is a ROS package with the ROS node *"finder"*. It is implemented in a turlebot receiving Kinect rgb image messages, and broadcasts the frames indicating whether the turtlebot finds the light switch in the room. The topic name is *'id_pub'*, and the message type is *std_msgs/String*.

1. If it finds the switch for Light A, it broadcast *'0'*;
2. If it finds the switch for Light B, it broadcast *'1'*;
3. If no switches are found, it broadcast *'-1'*;


#### Usage:
1. run the kinect package to publish rgb images

   *roslaunch openni_launch openni.launch* **OR** *roslaunch freenect_launch freenect.launch*

2. run the switch_finder package

   *rosrun switch_finder finder*



#### Dependence:
1. OpenCV
2. Kinect
