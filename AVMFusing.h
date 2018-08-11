#ifndef AVMFUSING_H
#define AVMFUSING_H

#include "ImageTransform.h"

class AVMFusing
{
public:
	Mat imageFused;
	AVMFusing(XmlFileState xmlParameters, ConfigParameters iniParameters);
	int creatBGRFiles();
	int creatYUYVFiles();
	int creatUYVYFiles();

private:
	int parameterGenerate();
	int coordinateCalculate();
	int fundamentalTransform();
	int rectangleTransform();
	int triangleTransform();
	int getBGRParametersFiles();
	int getYUYVParametersFiles();
	int getUYVYParametersFiles();
	int coordinateTransform();
	int getBGRFiles();
	int getYUYVFiles();
	int getUYVYFiles();


private:
	int testFlag;

	int sourceImageWidth;
	int sourceImageHeight;
	int targetImageWidth;
	int targetImageHeight;

	int rectangleRange[4][2][2];
	int triangleRange[4][2];
	float rectangleSlope[4];
	float triangleSlope[4];
	float rectangleExcursion[4][4];
	float triangleExcursion[4][2];
	XmlFileState AVMParamters;
	ImageTransform *imageTransformation[4];
	int pointNumber[4][5];

	std::unique_ptr<int[]> transformCoordinate[4][2];
	std::unique_ptr<int[]> fundamentalCoordinate[4][4];
	std::unique_ptr<int[]> rectangleCoordinate[4][6];
	std::unique_ptr<int[]> triangleCoordinate[4][6];

	std::unique_ptr<float[]> rectangleWeight[4];
	std::unique_ptr<float[]> triangleWeight[4];

	string BGRFundamentalPath[4];
	string BGRRectanglePath[7];
	string BGRTrianglePath[7];

	string YUYVFundamentalPath[2];
	string YUYVRectanglePath[4];
	string YUYVTrianglePath[4];

	string UYVYFundamentalPath[2];
	string UYVYRectanglePath[4];
	string UYVYTrianglePath[4];

	string BGRNumberPath;
	string YUYVNumberPath;
	string UYVYNumberPath;
};


#endif