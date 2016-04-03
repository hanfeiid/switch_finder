// detectId.cpp

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <switch_finder/ARTag.h>

// This function tries to find a black square in the image.
std::vector<cv::Point2f> findSquare(cv::Mat imageInput)
{
	// Convert to gray if input is color.
	cv::Mat	imageInputGray;
	if (imageInput.channels() == 3)
		cv::cvtColor(imageInput, imageInputGray, cv::COLOR_BGR2GRAY);
	else
		imageInputGray = imageInput;

	// Do adaptive threshold ... this compares each pixel to a local
	// mean of the neighborhood.  The result is a binary image, where
	// dark areas of the original image are now white (1's).
	cv::Mat imageThresh;
	adaptiveThreshold(imageInputGray,
		imageThresh,				// output thresholded image
		255,					// output value where condition met
		cv::ADAPTIVE_THRESH_GAUSSIAN_C,	// local neighborhood
		cv::THRESH_BINARY_INV,		// threshold_type - invert
		31,					// blockSize (any large number)
		0);					// a constant to subtract from mean


	// Apply morphological operations to get rid of small (noise) regions
	cv::Mat structuringElmt = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::Mat imageOpen;
	morphologyEx(imageThresh, imageOpen, cv::MORPH_OPEN, structuringElmt);
	cv::Mat imageClose;
	morphologyEx(imageOpen, imageClose, cv::MORPH_CLOSE, structuringElmt);


	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(
		imageClose,				// input image (is destroyed)
		contours,				// output vector of contours
		hierarchy,				// hierarchical representation
		CV_RETR_CCOMP,			// retrieve all contours
		CV_CHAIN_APPROX_NONE);	// all pixels of each contours

	// Iterate through all the top-level contours and find squares.
	std::vector<cv::Point2f> squareCorners;		// this will hold corners
	for (int i = 0; i < (int)contours.size(); i++) {
		// Contour should be greater than some minimum area
		double a = contourArea(contours[i]);
		if (!(a > 100))	continue;

		// Reject the ith contour if it doesn't have a child inside.
		if (hierarchy[i][2] < 0)	continue;

		// Approximate contour by a polygon.
		std::vector<cv::Point> approxCurve;
		// Maximum allowed distance between the original curve and its approximation.
		double eps = contours[i].size() * 0.01;
		cv::approxPolyDP(contours[i], approxCurve, eps, true);

		// We interested only in polygons that contain only four points.
		if (approxCurve.size() != 4)	continue;

		// Ok, I think we have a square! Create the list of corner points.
		for (int j = 0; j < 4; j++)
			squareCorners.push_back((approxCurve[j]));

		// Sort the points in counter-clockwise order. Trace a line between the
		// first and second point. If the third point is on the right side, then
		// the points are anticlockwise.
		cv::Point v1 = squareCorners[1] - squareCorners[0];
		cv::Point v2 = squareCorners[2] - squareCorners[0];
		double o = (v1.x * v2.y) - (v1.y * v2.x);
		if (o < 0.0)
			std::swap(squareCorners[1], squareCorners[3]);

		break;
	}

	return squareCorners;
}

// This function tries to read the bit pattern of a marker.
void readBitMatrix(cv::Mat imageMarker, int cellSize, int bitMatrix[10][10])
{
	// Threshold the marker image.
	cv::threshold(imageMarker,
		imageMarker,		// output thresholded image
		0,					// threshold value (not used; Otsu will compute)
		255,				// output value
		cv::THRESH_OTSU | cv::THRESH_BINARY);

	for (int iy = 0; iy < 10; iy++) {
		for (int ix = 0; ix < 10; ix++) {
			int x = ix * cellSize;
			int y = iy * cellSize;
			cv::Mat cell = imageMarker(cv::Rect(x, y, cellSize, cellSize));

			int nZ = cv::countNonZero(cell);

			if (nZ > (cellSize*cellSize) / 2)
				bitMatrix[iy][ix] = 1;
			else
				bitMatrix[iy][ix] = 0;
		}
	}
}

// This function tries to detect the artag info in the image.
int detectArtag(cv::Mat &imageInput, cv::Point2f &origin, cv::Point2f &z_axis)
{
	if (! imageInput.data) return -1;
	// Camera intrinsic matrix
	double K_[3][3] =	{ { 675, 0, 320 },
						{ 0, 675, 240 },
						{ 0, 0, 1 } };
	cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
	cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F); // distortion coeffs

	ARTag arTag;

	cv::Mat imageInputGray;
	cv::cvtColor(imageInput, imageInputGray, cv::COLOR_BGR2GRAY);
	std::vector<cv::Point2f> squareCorners;
	squareCorners = findSquare(imageInputGray);
	if (squareCorners.size() == 0)
	{
		cv::imshow("image", imageInput);
		cv::waitKey(10);  // Wait for 10 ms
		ROS_INFO("The id is [-1]");
		return -1;
	}
	// Draw contour as a sequence of line segments.
	cv::Scalar color = cv::Scalar(0, 0, 255);
	for (int j = 0; j < 4; j++)
	{
		cv::Point p1 = squareCorners[j];
		cv::Point p2 = squareCorners[(j + 1) % 4];
		cv::line(imageInput, p1, p2,
		  color,
		  2, // thickness
		  8); // line connectivity
	}

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
	if (!arTag.identifyMarker(bitMatrix, id, squareCorners))
	{
		cv::imshow("image", imageInput); // Show the image.
		cv::waitKey(10);
		ROS_INFO("The id is [-1]");
		return -1;
	}
	ROS_INFO("The id is [%d]", id);

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
		std::vector<cv::Point3f> pointsAxes;
		std::vector<cv::Point2f> pointsImage;

		// Draw the xyz coordinate axes on the image.
		pointsAxes.push_back(cv::Point3f(0, 0, 0)); // origin
		pointsAxes.push_back(cv::Point3f(1, 0, 0)); // x axis
		pointsAxes.push_back(cv::Point3f(0, 1, 0)); // y axis
		pointsAxes.push_back(cv::Point3f(0, 0, 1)); // z axis

		cv::projectPoints(pointsAxes, rotVec, transVec, K, dist, pointsImage);

		line(imageInput, pointsImage[0], pointsImage[1], cv::Scalar(0, 0, 255), 2);
		line(imageInput, pointsImage[0], pointsImage[2], cv::Scalar(0, 255, 0), 2);
		line(imageInput, pointsImage[0], pointsImage[3], cv::Scalar(255, 0, 0), 2);

		origin = pointsImage[0];
		z_axis = pointsImage[3];
	}

	cv::imshow("image", imageInput);	// show image
	cv::waitKey(10);
	return id;
}
