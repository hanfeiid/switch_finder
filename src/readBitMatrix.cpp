#include "opencv2/opencv.hpp"


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
