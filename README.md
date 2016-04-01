# switch_finder

switch_finder is a ROS package with the ROS node "finder". It is implemented in a turlebot receiving Kinect rgb image messages, and broadcasts the frames indicating whether the turtlebot finds the light switch in the room. The topic name is 'id_pub', and the message type is std_msgs/String.

1. If it finds the switch for Light A, it broadcast '0';
2. If it finds the switch for Light B, it broadcast '1';
3. If no switches are found, it broadcast '-1';
