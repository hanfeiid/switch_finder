// imgSubAndTagPub.cpp

#include <std_msgs/String.h>
#include <switch_finder/imgSubAndTagPub.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <switch_finder/point.h>
#include <switch_finder/detectId.h>

ImgSubAndTagPub::ImgSubAndTagPub()
{
	// image topic to subscribe
	image_transport::ImageTransport it(n_);
	sub_ = it.subscribe("camera/rgb/image_color", 1, &ImgSubAndTagPub::imageCallback, this);
	//sub_ = it.subscribe("cv_camera/image_raw", 1, &ImgSubAndTagPub::imageCallback, this);
	// id to publish
	id_pub_ = n_.advertise<std_msgs::String>("switch_finder/id_pub",1);
	// origin coordinate to publish
	origin_pub_ = n_.advertise<switch_finder::point>("switch_finder/origin_pub", 1);
	// z_axis coordinate to publish
	z_axis_pub_ = n_.advertise<switch_finder::point>("switch_finder/z_axis_pub", 1);
}

void ImgSubAndTagPub::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
	try
	{
		// change ros image message to opencv format
		cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Point2f origin(-1.0, -1.0);
		cv::Point2f z_axis(-1.0, -1.0);

		// detect the artag
		int id = detectArtag(frame, origin, z_axis);

		// broadcast the artag info, including id, the origin and z-axis coordinate
		if(ros::ok())
		{
			std_msgs::String id_msg;
			std::stringstream ss;
			ss << id;
			id_msg.data = ss.str();
			id_pub_.publish(id_msg);

			switch_finder::point origin_msg;
			origin_msg.x = origin.x;
			origin_msg.y = origin.y;
			origin_pub_.publish(origin_msg);

			switch_finder::point z_axis_msg;
			z_axis_msg.x = z_axis.x;
			z_axis_msg.y = z_axis.y;
			z_axis_pub_.publish(z_axis_msg);

			//ros::spinOnce();
			//loop_rate.sleep();
		}
	}

	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}
