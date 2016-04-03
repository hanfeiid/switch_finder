// ARTag.h

const int NPATTERNS = 2;
const int N = 6;		// patterns are NxN bits

class ARTag {

	int bits[NPATTERNS][N*N] = {
		{
			0, 0, 1, 1, 1, 0,
			0, 1, 1, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1,
			0, 0, 1, 1, 0, 0,
			0, 1, 0, 0, 1, 0
		},	// For Light Switch A, marked as '0'
		{
			0, 0, 1, 1, 0, 0,
			0, 1, 0, 1, 0, 0,
			1, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 1, 1,
			0, 1, 1, 0, 0, 0,
			0, 1, 0, 1, 0, 0
		}	// For Light Switch B, marked as '1'
	};

	int bits90[NPATTERNS][N*N];
	int bits180[NPATTERNS][N*N];
	int bits270[NPATTERNS][N*N];

public:
	////////////  Constructor //////////////
	ARTag();

	////////////  Methods  /////////////////////
	bool identifyMarker(int bitMatrix[10][10], int& id, std::vector<cv::Point2f>& squareCorners);

};
