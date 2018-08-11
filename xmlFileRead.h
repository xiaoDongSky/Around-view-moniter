#ifndef XMLFILEREAD_H
#define XMLFILEREAD_H
#include <stdio.h>
#include <tchar.h>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
struct CameraParameter
{
	Mat intrinsicMatrix;
	Mat homographyMatrix;
	//Mat affMatrix;
	Vec4f distortCoffs;
	Vec2f homographyPoint;
	int colorDiff[3];
	float ratio;
};

struct FusingParameter
{
	Vec2f jiontPoint;
	int translationCoordinate[8];
};

struct XmlFileState
{
	CameraParameter cameraState[4];
	FusingParameter fusingState;
};
int xmlFileRead(XmlFileState &xmlFileState,string xmlPath0,string xmlPath1,string xmlPath2,string xmlPath3);

#endif