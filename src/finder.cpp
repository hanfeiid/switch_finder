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

int detectArtag(cv::Mat &imageInput)
{
  
  ARTag arTag;

  // Camera intrinsic matrix
  double K_[3][3] =
  { { 675, 0, 320 },
  { 0, 675, 240 },
  { 0, 0, 1 } };
  cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
  cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F); // distortion coeffs

  cv::Mat imageInputGray;
  cv::cvtColor(imageInput, imageInputGray, cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> squareCorners;
  squareCorners = findSquare(imageInputGray);
  if (squareCorners.size() == 0) {
    //cv::imshow("My Image", imageInput);
    // Wait for xx ms (0 means wait until a keypress)
    cv::waitKey(10);
    ROS_INFO("The id is [-1]");
    return -1; // hit ESC (ascii code 27) to quit
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

  //cv::imshow("My Image", imageInput);
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
    // Show the image.
    //cv::imshow("My Image", imageInput);
    ROS_INFO("The id is [%d]", id);
    // Wait for xx ms (0 means wait until a keypress)
    return -1; // hit ESC (ascii code 27) to quit
  }
  ROS_INFO("The id is [%d]", id);
  // Display the marker id number.
  //
  //char sz[80];
  //sprintf_s(sz, "%d", id);
  //putText(imageInput, sz, cv::Point(320, 240),
  //  cv::FONT_HERSHEY_PLAIN, // font face
  //  3.0, // font scale
  //  cv::Scalar(0, 255, 255), // font color
  //  3);
  //

  /*
  // Refine corner locations.
  cv::cornerSubPix(imageInputGray,
    squareCorners,
    cv::Size(5, 5),
    cv::Size(-1, -1),
    cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));
  // Create a list of 3D marker corner points.
  std::vector<cv::Point3f> marker3D;
  marker3D.push_back(cv::Point3f(-1.0f, -1.0f, 0));
  marker3D.push_back(cv::Point3f(+1.0f, -1.0f, 0));
  marker3D.push_back(cv::Point3f(+1.0f, +1.0f, 0));
  marker3D.push_back(cv::Point3f(-1.0f, +1.0f, 0));
  // Compute pose of marker with respect to camera.
  cv::Mat rotVec, transVec;
  bool foundPose = cv::solvePnP(marker3D, squareCorners,
    K, // intrinsic camera parameter matrix
    dist, // distortion coefficients
    rotVec, transVec); // output rotation and translation

  if (foundPose)
  {
    std::vector<cv::Point3d> pointsAxes;
    std::vector<cv::Point2d> pointsImage;
    
    // Draw the xyz coordinate axes on the image.
    pointsAxes.push_back(cv::Point3d(0, 0, 0)); // origin
    pointsAxes.push_back(cv::Point3d(1, 0, 0)); // x axis
    pointsAxes.push_back(cv::Point3d(0, 1, 0)); // y axis
    pointsAxes.push_back(cv::Point3d(0, 0, 1)); // z axis

    cv::projectPoints(pointsAxes, rotVec, transVec, K, dist, pointsImage);

    line(imageInput, pointsImage[0], pointsImage[1], cv::Scalar(0, 0, 255), 2);
    line(imageInput, pointsImage[0], pointsImage[2], cv::Scalar(0, 255, 0), 2);
    line(imageInput, pointsImage[0], pointsImage[3], cv::Scalar(255, 0, 0), 2);


    // Find the projected image position of the power button
    std::vector<cv::Point3d> pointAxesButton; // the button position in 3D world coordinate
    std::vector<cv::Point2d> pointsImageButton; // the button position in 2D image coordinate
    if (id == 0)  // for oscilloscope
    {
      pointAxesButton.push_back(cv::Point3d(-11.5, 1.75, 0));
    }
    else if (id == 1) // for multimeter
    {
      pointAxesButton.push_back(cv::Point3d(-3.5, 3.0, 2.75));
    }
    cv::projectPoints(pointAxesButton, rotVec, transVec, K, dist, pointsImageButton);
    CvPoint center = pointsImageButton[0];
    circle(imageInput, center, 20, cv::Scalar(0, 255, 255), 2);

  }

  // Show the image.
  //cv::imshow("My Image", imageInput);
  */
  return id;
  
}

// reader node to show camera images
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    //cv::imshow("kinect view", frame);
    int id = detectArtag(frame);

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

