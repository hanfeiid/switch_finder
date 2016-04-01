# switch_finder
The switch_finder broadcasts the frames indicating whether the turtlebot finds the light switch in the room. The topic name is 'id_pub', and the type of message is std_msgs/String.

If it finds the switch for Light A, it broadcast '0';
If it finds the switch for Light B, it broadcast '1';
If no switches are found, it broadcast '-1';
