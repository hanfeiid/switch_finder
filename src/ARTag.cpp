// ARTag.cpp

#include <opencv2/opencv.hpp>
#include <switch_finder/ARTag.h>

// A private function to check if two NxN bit patterns are equal.
bool checkEqual(int b1[], int b2[])
{
	for (int i = 0; i < N*N; i++)
		if (b1[i] != b2[i])
			return false;
	return true;
}



// The constructor for the ARTag class.  It generates the rotated
// versions of the bit patterns.
ARTag::ARTag() {
	// Rotate by 90 degrees.
	for (int n = 0; n < NPATTERNS; n++)
		for (int i = 0; i < N; i++)
			for (int j = 0; j < N; j++)
				bits90[n][i*N + j] = bits[n][(N - j - 1)*N + i];

	// Rotate again by 90 degrees to get 180 degree rotation.
	for (int n = 0; n < NPATTERNS; n++)
		for (int i = 0; i < N; i++)
			for (int j = 0; j < N; j++)
				bits180[n][i*N + j] = bits90[n][(N - j - 1)*N + i];


	// Rotate again by 90 degrees to get 270 degree rotation.
	for (int n = 0; n < NPATTERNS; n++)
		for (int i = 0; i < N; i++)
			for (int j = 0; j < N; j++)
				bits270[n][i*N + j] = bits180[n][(N - j - 1)*N + i];
}



// This function takes as input a 10x10 bit matrix, extracted from an image.
// It checks the bit matrix against each of the pre-stored patterns.  If it matches,
// it returns true, and also the id of the pattern that matched.
//
// Also input is a list of image corner points.  If the input pattern is rotated,
// the function rotates the corner points to put them in "canonical" form; ie, the
// same order as the pre-stored pattern.
bool ARTag::identifyMarker(int bitMatrix[10][10], int& id, std::vector<cv::Point2f>& squareCorners)
{
	// First, make sure the borders are all black.
	for (int i = 0; i < 10; i++)
		for (int j = 0; j < 2; j++)
			if (bitMatrix[i][j] != 0)
				return false;
	for (int i = 0; i < 10; i++)
		for (int j = 8; j < 10; j++)
			if (bitMatrix[i][j] != 0)
				return false;
	for (int i = 0; i < 2; i++)
		for (int j = 2; j < 8; j++)
			if (bitMatrix[i][j] != 0)
				return false;
	for (int i = 8; i < 10; i++)
		for (int j = 2; j < 8; j++)
			if (bitMatrix[i][j] != 0)
				return false;

	// Get the inner 6x6 bit pattern
	int bitsInput[N*N];
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			bitsInput[i*N + j] = bitMatrix[i + 2][j + 2];

	// Compare the input bits against each of the stored patterns.
	for (id = 0; id < NPATTERNS; ++id) {
		if (checkEqual(bitsInput, bits[id])) {
			return true;
		}
		if (checkEqual(bitsInput, bits90[id])) {
			// Rotate the points one position left.
			std::rotate(squareCorners.begin(), squareCorners.begin() + 1, squareCorners.end());
			return true;
		}
		if (checkEqual(bitsInput, bits180[id])) {
			// Rotate the points 2 positions left.
			std::rotate(squareCorners.begin(), squareCorners.begin() + 2, squareCorners.end());
			return true;
		}
		if (checkEqual(bitsInput, bits270[id])) {
			// Rotate the points 3 positions left.
			std::rotate(squareCorners.begin(), squareCorners.begin() + 3, squareCorners.end());
			return true;
		}
	}

	// Didn't match any of them.
	return false;
}
