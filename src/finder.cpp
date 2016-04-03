// finder.cpp

#include <switch_finder/imgSubAndTagPub.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "switch_finder");
	ImgSubAndTagPub ImgSubAndTagPub_Obj;

	ros::spin();
	return 0;
}
