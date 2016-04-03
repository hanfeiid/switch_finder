// detectId.h

std::vector<cv::Point2f> findSquare(cv::Mat imageInput);
void readBitMatrix(cv::Mat imageMarker, int cellSize, int bitMatrix[10][10]);
int detectArtag(cv::Mat &imageInput, cv::Point2f &origin, cv::Point2f &z_axis);
