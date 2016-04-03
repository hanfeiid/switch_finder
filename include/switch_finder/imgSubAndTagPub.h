// imgSubAndTagPub.h

#ifndef IMG_SUB_AND_TAG_PUB_FILE_H
#define IMG_SUB_AND_TAG_PUB_FILE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ImgSubAndTagPub
{
public:
	ImgSubAndTagPub();
	void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
	ros::NodeHandle n_;
	image_transport::Subscriber sub_;
	ros::Publisher id_pub_;
	ros::Publisher origin_pub_;
	ros::Publisher z_axis_pub_;
};

#endif	/* IMG_SUB_AND_TAG_PUB_FILE_H */
