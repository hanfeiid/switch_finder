#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include "switch_finder/ARTag.h"

std::vector<cv::Point2f> findSquare(cv::Mat imageInput);
void readBitMatrix(cv::Mat imageMarker, int cellSize, int bitMatrix[10][10]);

// detect the artag in the image
int detectArtag(cv::Mat &imageInput)
{
  ARTag arTag;

  cv::Mat imageInputGray;
  cv::cvtColor(imageInput, imageInputGray, cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> squareCorners;
  squareCorners = findSquare(imageInputGray);
  if (squareCorners.size() == 0) {
    cv::waitKey(10);  // Wait for 10 ms
    ROS_INFO("The id is [-1]");
    return -1;
  }
  // Draw contour as a sequence of line segments.
  cv::Scalar color = cv::Scalar(0, 0, 255);
  for (int j = 0; j < 4; j++) {
    cv::Point p1 = squareCorners[j];
    cv::Point p2 = squareCorners[(j + 1) % 4];
    cv::line(imageInput, p1, p2,
      color,
      2, // thickness
      8); // line connectivity
  }

  cv::waitKey(10);

  // Create a list of "ortho" square corner points.
  std::vector<cv::Point2f> squareOrtho;
  squareOrtho.push_back(cv::Point2f(0, 0));
  squareOrtho.push_back(cv::Point2f(100, 0));
  squareOrtho.push_back(cv::Point2f(100, 100));
  squareOrtho.push_back(cv::Point2f(0, 100));
  // Find the perspective transfomation that brings current marker to rectangular form.
  cv::Mat M = cv::getPerspectiveTransform(squareCorners, squareOrtho);
  // Transform image to get an orthophoto square image.
  cv::Mat imageSquare;
  const int cellSize = 10;
  cv::Size imageSquareSize(10 * cellSize, 10 * cellSize);
  cv::warpPerspective(imageInputGray, imageSquare, M, imageSquareSize);
  cv::imshow("Marker", imageSquare);

  // Read the bit matrix.  Markers are divided into 10x10 cells,
  // of which the inner 6x6 belongs to marker info.  The external border is
  // 2 cells wide, and should be entirely black.
  int bitMatrix[10][10];
  std::string bitStr;
  readBitMatrix(imageSquare, cellSize, bitMatrix);

  // Identify the marker.
  int id = -1; // marker id
  if (!arTag.identifyMarker(bitMatrix, id, squareCorners)) {
    //cv::imshow("My Image", imageInput); // Show the image.
    ROS_INFO("The id is [%d]", id);
    return -1;
  }
  ROS_INFO("The id is [%d]", id);
  
  return id;
}


// imageCallback() functionality:
// 1. receive and read the Kinect rgb image message
// 2. detect artag
// 3. broadcast the artag infomation
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // change ros image message to opencv format
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    // detect the artag
    int id = detectArtag(frame);

    // broadcast the artag info
    ros::NodeHandle n_id;
    ros::Publisher id_pub = n_id.advertise<std_msgs::String>("id_pub", 1000);
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
      std_msgs::String id_msg;
      std::stringstream ss;
      ss << id;
      id_msg.data = ss.str();

      id_pub.publish(id_msg);
      ros::spinOnce();
      loop_rate.sleep();
      cv::waitKey(10);
    }
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "switch_finder");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color", 1, imageCallback);
  ros::spin();
}

