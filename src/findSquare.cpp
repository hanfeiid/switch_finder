#include "opencv2/opencv.hpp"


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