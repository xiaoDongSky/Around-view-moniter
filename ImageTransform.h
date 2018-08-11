#ifndef IMAGETRANSFORM_H
#define IMAGETRANSFORM_H
#include <math.h>
#include <memory>
#include "xmlFileRead.h"
#include "configuration.h"
#define AVM_TEST

class ImageTransform
{
public:
	enum CameraPosition { front = 1, left, right, back };
	Mat imageCorrect;
	//parameter from xubin
	//flag test or no?
	ImageTransform(CameraParameter parameter, CameraPosition position, int imageWidth, int imageHeight,int flag);
	int getImageTransformation(int *rowCoordinateTrans, int *colCoordinateTrans);
#ifdef AVM_TEST
	Mat imageSource;
	Mat imageHomography;
	Mat imageTest;
#endif

private:
	int testFlag;
	int sourceImageWidth;
	int sourceImageHeight;
	int distortionCorrection();
	int matrixTransformation();
	int homographyTransformation();
	int coordinateTransorm();

	CameraPosition AVMPosition;
	Size sourceImageSize;
	CameraParameter AVMParameter;
	Mat homographyMatrix;

	std::unique_ptr<int[]> correctCoordinateDiff[2];
	std::unique_ptr<int[]> homographyCoordinateDiff[2];
	std::unique_ptr<int[]> coordinateDiff[2];


};

#endif