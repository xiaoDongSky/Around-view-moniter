#include "ImageTransform.h"
#define AVM_TEST

ImageTransform::ImageTransform(CameraParameter parameter, CameraPosition position, int imageWidth, int imageHeight, int flag) : AVMParameter(parameter), AVMPosition(position)
{
	testFlag = flag;
	sourceImageWidth = imageWidth;
	sourceImageHeight = imageHeight;

	correctCoordinateDiff[0].reset(new int[sourceImageWidth*sourceImageHeight]);
	correctCoordinateDiff[1].reset(new int[sourceImageWidth*sourceImageHeight]);

	homographyCoordinateDiff[0].reset(new int[imageHomographyWidth*imageHomographyHeight]);
	homographyCoordinateDiff[1].reset(new int[imageHomographyWidth*imageHomographyHeight]);

	coordinateDiff[0].reset(new int[imageHomographyWidth*imageHomographyHeight]);
	coordinateDiff[1].reset(new int[imageHomographyWidth*imageHomographyHeight]);

	sourceImageSize.height = sourceImageHeight;
	sourceImageSize.width = sourceImageWidth;

	if (testFlag == 1)
	{
		imageCorrect.create(sourceImageSize, CV_8UC3);
		imageTest.create(sourceImageSize, CV_8UC3);

		if ((AVMPosition == front) || (AVMPosition == back))
		{
			imageHomography.create(imageHomographyHeight, imageHomographyWidth, CV_8UC3);
		}
		else
			imageHomography.create(imageHomographyWidth, imageHomographyHeight, CV_8UC3);

		string imageName;
		if (AVMPosition == front)
			imageName = "..\\..\\Parameter\\0.jpg";
		else if (AVMPosition == left)
			imageName = "..\\..\\Parameter\\1.jpg";
		else if (AVMPosition == right)
			imageName = "..\\..\\Parameter\\2.jpg";
		else
			imageName = "..\\..\\Parameter\\3.jpg";

		imageSource = imread(imageName);
	}
	
}

int ImageTransform::getImageTransformation(int *rowCoordinateTrans, int *colCoordinateTrans)
{
	matrixTransformation();
	distortionCorrection();


	homographyTransformation();
	coordinateTransorm();
	memcpy(rowCoordinateTrans, coordinateDiff[1].get(), imageHomographyWidth*imageHomographyHeight * sizeof(int));
	memcpy(colCoordinateTrans, coordinateDiff[0].get(), imageHomographyWidth*imageHomographyHeight * sizeof(int));
	return 1;
}

int ImageTransform::distortionCorrection()
{
	Mat mapx(sourceImageHeight, sourceImageWidth, CV_32FC1);
	Mat mapy(sourceImageHeight, sourceImageWidth, CV_32FC1);
	Mat new_intrinsic_matrix;
	AVMParameter.intrinsicMatrix.copyTo(new_intrinsic_matrix);
	new_intrinsic_matrix.at<float>(0, 0) *= AVMParameter.ratio;
	new_intrinsic_matrix.at<float>(1, 1) *= AVMParameter.ratio;
	fisheye::initUndistortRectifyMap(AVMParameter.intrinsicMatrix, AVMParameter.distortCoffs, Matx33f::eye(), new_intrinsic_matrix, sourceImageSize, CV_32FC1, mapx, mapy);

	for (int row = 0; row<sourceImageHeight; row++)
		for (int col = 0; col < sourceImageWidth; col++)
		{
			int imagey = max(float(0), mapy.at<float>(row, col));
			imagey = min(sourceImageHeight - 1, imagey);
			int imagex = max(float(0), mapx.at<float>(row, col));
			imagex = min(sourceImageWidth - 1, imagex);
			if (testFlag == 1)
			{
				imageCorrect.at<Vec3b>(row, col) = imageSource.at<Vec3b>(imagey, imagex);
			}
			
			correctCoordinateDiff[0][row*sourceImageWidth + col] = imagex;
			correctCoordinateDiff[1][row*sourceImageWidth + col] = imagey;
		}

	return 1;
}

int ImageTransform::homographyTransformation()
{
	int rowMax, colMax;
	if ((AVMPosition == front) || (AVMPosition == back))
	{
		rowMax = imageHomographyHeight;
		colMax = imageHomographyWidth;
	}
	else
	{
		rowMax = imageHomographyWidth;
		colMax = imageHomographyHeight;
	}

	for (int row = 0; row < rowMax; row++)
		for (int col = 0; col < colMax; col++)
		{
			float normalization = (homographyMatrix.at<float>(2, 0) * col + homographyMatrix.at<float>(2, 1) * row + homographyMatrix.at<float>(2, 2));
			float col2 = ((homographyMatrix.at<float>(0, 0) * col + homographyMatrix.at<float>(0, 1) * row + homographyMatrix.at<float>(0, 2))) / normalization;
			float row2 = ((homographyMatrix.at<float>(1, 0) * col + homographyMatrix.at<float>(1, 1) * row + homographyMatrix.at<float>(1, 2))) / normalization;

			int imagex = max(float(0), col2);
			imagex = min(sourceImageWidth - 1, imagex);

			int imagey = max(float(0), row2);
			imagey = min(sourceImageHeight - 1, imagey);
			if (testFlag == 1)
			{
				imageHomography.at<Vec3b>(row, col) = imageCorrect.at<Vec3b>(imagey, imagex);
			}
			homographyCoordinateDiff[0][row*colMax + col] = imagex;
			homographyCoordinateDiff[1][row*colMax + col] = imagey;
		}
	//imshow("a", imageCorrect);
	//imshow("b", imageHomography);
	//cvWaitKey();

	return 1;
}

int ImageTransform::matrixTransformation()
{
	float H1[9] = { 1, 0, -AVMParameter.homographyPoint[0], 0, 1, -AVMParameter.homographyPoint[1], 0, 0, 1 };
	Mat tansfromMatrix(3, 3, CV_32FC1, H1);

	float frontPosition[9] = { 1,0,0,0,1,0,0,0,1 };
	float leftPosition[9] = { 0, 1, 0, -1, 0, 1200 - 1, 0, 0, 1 };
	float rightPosition[9] = { 0, -1, 1200 - 1, 1, 0, 0, 0, 0, 1 };
	float backPosition[9] = { -1, 0, 1200 - 1, 0, -1, 1200 - 1, 0, 0, 1 };
	Mat positionMatrix(3, 3, CV_32FC1);
	if (AVMPosition == front)
	{
		positionMatrix = Mat(3, 3, CV_32FC1, frontPosition).clone();
	}
	else if (AVMPosition == left)
	{
		positionMatrix = Mat(3, 3, CV_32FC1, leftPosition).clone();
	}
	else if (AVMPosition == right)
	{
		positionMatrix = Mat(3, 3, CV_32FC1, rightPosition).clone();
	}
	else if (AVMPosition == back)
	{
		positionMatrix = Mat(3, 3, CV_32FC1, backPosition).clone();
	}

	Mat matrixMul = tansfromMatrix *	AVMParameter.homographyMatrix;
	//matrixMul = AVMParameter.affMatrix * matrixMul;
	//matrixMul = tansfromMatrix * matrixMul;
	homographyMatrix = matrixMul.inv();

	return 1;
}

int ImageTransform::coordinateTransorm()
{
	for (int i = 0; i < imageHomographyHeight*imageHomographyWidth; i++)
	{
		coordinateDiff[0][i] = correctCoordinateDiff[0][homographyCoordinateDiff[0][i] + homographyCoordinateDiff[1][i] * sourceImageWidth];
		coordinateDiff[1][i] = correctCoordinateDiff[1][homographyCoordinateDiff[0][i] + homographyCoordinateDiff[1][i] * sourceImageWidth];
	}
	return 1;
}
