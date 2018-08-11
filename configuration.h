#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <boost/property_tree/ptree.hpp>  
#include <boost/property_tree/ini_parser.hpp>  
#include <string>
#include <iostream> 
using namespace std;

struct ConfigParameters
{
	int sourceImageWidth;
	int sourceImageHeight;
	int targetImageWidth;
	int targetImageHeight;
	int testFlag;
	string colorSpace;
	string BGRNumberPath;
	string YUYVNumberPath;
	string UYVYNumberPath;
	string xmlPath[4];
	string BGRFundamentalPath[4];
	string BGRRectanglePath[7];
	string BGRTrianglePath[7];

	string YUYVFundamentalPath[2];
	string YUYVRectanglePath[4];
	string YUYVTrianglePath[4];

	string UYVYFundamentalPath[2];
	string UYVYRectanglePath[4];
	string UYVYTrianglePath[4];
};

#define imageHomographyWidth 720
#define imageHomographyHeight 480
#define IMAGE_POSITION_X 0
#define IMAGE_POSITION_Y 0
#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 720

void configurationReading(ConfigParameters &parameters);


#endif
