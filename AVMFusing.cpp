#include "AVMFusing.h"

AVMFusing::AVMFusing(XmlFileState xmlParameters, ConfigParameters iniParameters) :AVMParamters(xmlParameters)
{
	testFlag = iniParameters.testFlag;
	sourceImageWidth = iniParameters.sourceImageWidth;
	sourceImageHeight = iniParameters.sourceImageHeight;
	targetImageWidth = iniParameters.targetImageWidth;
	targetImageHeight = iniParameters.targetImageHeight;

	for (int i = 0; i < 4; i++)
	{
		BGRFundamentalPath[i] = iniParameters.BGRFundamentalPath[i];
	}
	for (int i = 0; i < 7; i++)
	{
		BGRRectanglePath[i] = iniParameters.BGRRectanglePath[i];
		BGRTrianglePath[i] = iniParameters.BGRTrianglePath[i];
	}
	for (int i = 0; i < 2; i++)
	{
		YUYVFundamentalPath[i] = iniParameters.YUYVFundamentalPath[i];
		UYVYFundamentalPath[i] = iniParameters.UYVYFundamentalPath[i];
	}
	for (int i = 0; i < 4; i++)
	{
		YUYVRectanglePath[i] = iniParameters.YUYVRectanglePath[i];
		YUYVTrianglePath[i] = iniParameters.YUYVTrianglePath[i];
		UYVYRectanglePath[i] = iniParameters.UYVYRectanglePath[i];
		UYVYTrianglePath[i] = iniParameters.UYVYTrianglePath[i];
	}

	BGRNumberPath = iniParameters.BGRNumberPath;
	YUYVNumberPath = iniParameters.YUYVNumberPath;
	UYVYNumberPath = iniParameters.UYVYNumberPath;

	if (testFlag == 1)
	{
		imageFused.create(targetImageHeight, targetImageWidth, CV_8UC3);
	}

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			fundamentalCoordinate[i][j].reset(new int[100000]);//50000
		}
	for (int i = 0; i < 4; i++)
	{
		rectangleWeight[i].reset(new float[60000]);
		triangleWeight[i].reset(new float[20000]);
		for (int j = 0; j < 6; j++)
		{
			rectangleCoordinate[i][j].reset(new int[60000]);
			triangleCoordinate[i][j].reset(new int[20000]);
		}
	}

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 5; j++)
		{
			pointNumber[i][j] = 0;
		}

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 2; j++)
		{
			transformCoordinate[i][j].reset(new int[imageHomographyHeight*imageHomographyWidth]);
		}

	imageTransformation[0] = new ImageTransform(xmlParameters.cameraState[0], ImageTransform::front, sourceImageWidth, sourceImageHeight, testFlag);
	imageTransformation[1] = new ImageTransform(xmlParameters.cameraState[1], ImageTransform::left, sourceImageWidth, sourceImageHeight, testFlag);
	imageTransformation[2] = new ImageTransform(xmlParameters.cameraState[2], ImageTransform::right, sourceImageWidth, sourceImageHeight, testFlag);
	imageTransformation[3] = new ImageTransform(xmlParameters.cameraState[3], ImageTransform::back, sourceImageWidth, sourceImageHeight, testFlag);
	imageTransformation[0]->getImageTransformation(transformCoordinate[0][0].get(), transformCoordinate[0][1].get());
	imageTransformation[1]->getImageTransformation(transformCoordinate[1][0].get(), transformCoordinate[1][1].get());
	imageTransformation[2]->getImageTransformation(transformCoordinate[2][0].get(), transformCoordinate[2][1].get());
	imageTransformation[3]->getImageTransformation(transformCoordinate[3][0].get(), transformCoordinate[3][1].get());
	/*
	imshow("1", imageTransformation[0]->imageHomography);
	imshow("2", imageTransformation[1]->imageHomography);
	imshow("3", imageTransformation[2]->imageHomography);
	imshow("4", imageTransformation[3]->imageHomography);
	cvWaitKey();
	*/
}

int AVMFusing::creatBGRFiles()
{
	parameterGenerate();
	coordinateCalculate();
	coordinateTransform();
	getBGRFiles();
	getBGRParametersFiles();
	if (testFlag == 1)
	{
		fundamentalTransform();
		rectangleTransform();
		triangleTransform();

		imshow("imageFused", imageFused);
		cvWaitKey();
	}
	
	return 1;
}

int AVMFusing::creatYUYVFiles()
{
	parameterGenerate();
	coordinateCalculate();
	coordinateTransform();
	getYUYVFiles();
	getYUYVParametersFiles();

	return 1;
}

int AVMFusing::creatUYVYFiles()
{
	parameterGenerate();
	coordinateCalculate();
	coordinateTransform();
	getUYVYFiles();
	getUYVYParametersFiles();
	if (testFlag == 1)
	{
		fundamentalTransform();
		rectangleTransform();
		triangleTransform();
		imwrite("out.bmp", imageFused);
		imshow("imageFused", imageFused);
		cvWaitKey();
	}
	return 1;
}

int AVMFusing::parameterGenerate()
{
	rectangleRange[0][0][0] = AVMParamters.fusingState.jiontPoint[0] - AVMParamters.fusingState.translationCoordinate[0];
	rectangleRange[0][0][1] = AVMParamters.fusingState.jiontPoint[0] - AVMParamters.fusingState.translationCoordinate[0] + targetImageWidth;
	rectangleRange[0][1][0] = AVMParamters.fusingState.jiontPoint[1] - AVMParamters.fusingState.translationCoordinate[1];
	rectangleRange[0][1][1] = imageHomographyHeight;

	rectangleRange[1][0][0] = AVMParamters.fusingState.jiontPoint[0];
	rectangleRange[1][0][1] = imageHomographyHeight;
	rectangleRange[1][1][0] = AVMParamters.fusingState.jiontPoint[1] - AVMParamters.fusingState.translationCoordinate[3];
	rectangleRange[1][1][1] = targetImageHeight - AVMParamters.fusingState.translationCoordinate[3] + AVMParamters.fusingState.jiontPoint[1];

	rectangleRange[2][0][0] = 0;
	rectangleRange[2][0][1] = targetImageWidth - AVMParamters.fusingState.translationCoordinate[4] + AVMParamters.fusingState.jiontPoint[0];
	rectangleRange[2][1][0] = AVMParamters.fusingState.jiontPoint[1] - AVMParamters.fusingState.translationCoordinate[5];
	rectangleRange[2][1][1] = targetImageHeight - AVMParamters.fusingState.translationCoordinate[5] + AVMParamters.fusingState.jiontPoint[1];

	rectangleRange[3][0][0] = AVMParamters.fusingState.jiontPoint[0] - AVMParamters.fusingState.translationCoordinate[6];
	rectangleRange[3][0][1] = AVMParamters.fusingState.jiontPoint[0] - AVMParamters.fusingState.translationCoordinate[6] + targetImageWidth;
	rectangleRange[3][1][0] = 0;
	rectangleRange[3][1][1] = targetImageHeight - AVMParamters.fusingState.translationCoordinate[7] + AVMParamters.fusingState.jiontPoint[1];

	triangleRange[0][0] = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[0];
	triangleRange[0][1] = AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[0];
	triangleRange[1][0] = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[3] + AVMParamters.fusingState.translationCoordinate[1];
	triangleRange[1][1] = AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[3];
	triangleRange[2][0] = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[5] + AVMParamters.fusingState.translationCoordinate[1];
	triangleRange[2][1] = AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[5];
	triangleRange[3][0] = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[6];
	triangleRange[3][1] = AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[6];

	rectangleSlope[0] = float((imageHomographyHeight - 1 + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.translationCoordinate[3])) / float((imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[0]));
	rectangleSlope[1] = float((imageHomographyHeight - 1 + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.translationCoordinate[5])) / float((AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[0] - (imageHomographyWidth - 1)));
	rectangleSlope[2] = float((AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[3] - imageHomographyWidth - 1)) / float(imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[6]);
	rectangleSlope[3] = float(imageHomographyWidth - 1 + AVMParamters.fusingState.translationCoordinate[5] - AVMParamters.fusingState.translationCoordinate[7]) / float(imageHomographyWidth - 1 - AVMParamters.fusingState.translationCoordinate[4] + AVMParamters.fusingState.translationCoordinate[6]);

	float excursion = AVMParamters.fusingState.translationCoordinate[3] - AVMParamters.fusingState.translationCoordinate[1];
	float rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[0] * rectangleSlope[0]);
	rectangleExcursion[0][0] = excursion + rectangleLength;
	rectangleExcursion[0][1] = excursion - rectangleLength;

	excursion = imageHomographyHeight - 1 - rectangleSlope[1] * (AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[0]);
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[1] * rectangleSlope[1]);
	rectangleExcursion[0][2] = excursion + rectangleLength;
	rectangleExcursion[0][3] = excursion - rectangleLength;

	excursion = -rectangleSlope[0] * AVMParamters.fusingState.translationCoordinate[0];
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[0] * rectangleSlope[0]);
	rectangleExcursion[1][0] = excursion + rectangleLength;
	rectangleExcursion[1][1] = excursion - rectangleLength;

	excursion = imageHomographyWidth - 1 - rectangleSlope[2] * AVMParamters.fusingState.translationCoordinate[6];
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[2] * rectangleSlope[2]);
	rectangleExcursion[1][2] = excursion + rectangleLength;
	rectangleExcursion[1][3] = excursion - rectangleLength;

	excursion = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[5] + AVMParamters.fusingState.translationCoordinate[1];
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[1] * rectangleSlope[1]);
	rectangleExcursion[2][0] = excursion + rectangleLength;
	rectangleExcursion[2][1] = excursion - rectangleLength;

	excursion = AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[5];
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[3] * rectangleSlope[3]);
	rectangleExcursion[2][2] = excursion + rectangleLength;
	rectangleExcursion[2][3] = excursion - rectangleLength;

	excursion = imageHomographyWidth - 1 + AVMParamters.fusingState.translationCoordinate[3] - AVMParamters.fusingState.translationCoordinate[7];
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[2] * rectangleSlope[2]);
	rectangleExcursion[3][0] = excursion + rectangleLength;
	rectangleExcursion[3][1] = excursion - rectangleLength;

	excursion = -rectangleSlope[3] * (AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[6]);
	rectangleLength = sqrt(50 * 50 + 50 * 50 * rectangleSlope[3] * rectangleSlope[3]);
	rectangleExcursion[3][2] = excursion + rectangleLength;
	rectangleExcursion[3][3] = excursion - rectangleLength;


	float x2 = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[0];
	float y1 = imageHomographyHeight - 1;
	float y2 = x2*rectangleSlope[0] + rectangleExcursion[0][1];
	float x1 = (y1 - rectangleExcursion[0][0]) / rectangleSlope[0];
	triangleSlope[0] = (y2 - y1) / (x2 - x1);
	triangleExcursion[0][0] = y1 - triangleSlope[0] * x1;

	x1 = AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[0];
	y1 = x1 * rectangleSlope[1] + rectangleExcursion[0][3];
	y2 = imageHomographyHeight - 1;
	x2 = (y2 - rectangleExcursion[0][2]) / rectangleSlope[1];
	triangleSlope[1] = (y2 - y1) / (x2 - x1);
	triangleExcursion[0][1] = y1 - triangleSlope[1] * x1;

	x1 = imageHomographyHeight - 1;
	y1 = x1 * rectangleSlope[0] + rectangleExcursion[1][1];
	y2 = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[3];
	x2 = (y2 - rectangleExcursion[1][0]) / rectangleSlope[0];
	triangleExcursion[1][0] = y1 - x1 * triangleSlope[0];

	y1 = AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[3];
	x1 = (y1 - rectangleExcursion[1][3]) / rectangleSlope[2];
	x2 = imageHomographyHeight - 1;
	y2 = x2* rectangleSlope[2] + rectangleExcursion[1][2];
	triangleSlope[2] = (y2 - y1) / (x2 - x1);
	triangleExcursion[1][1] = y1 - triangleSlope[2] * x1;

	x1 = 0;
	y1 = x1 * rectangleSlope[1] + rectangleExcursion[2][1];
	y2 = imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[5] + AVMParamters.fusingState.translationCoordinate[1];
	x2 = (y2 - rectangleExcursion[2][0]) / rectangleSlope[1];
	triangleExcursion[2][0] = y1 - triangleSlope[1] * x1;

	y1 = AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[5];
	x1 = (y1 - rectangleExcursion[2][3]) / rectangleSlope[3];
	x2 = 0;
	y2 = x2*rectangleSlope[3] + rectangleExcursion[2][2];
	triangleSlope[3] = (y2 - y1) / (x2 - x1);
	triangleExcursion[2][1] = y1 - triangleSlope[3] * x1;

	y1 = 0;
	x1 = (y1 - rectangleExcursion[3][1]) / rectangleSlope[2];
	x2 = 480 - AVMParamters.fusingState.translationCoordinate[6];
	y2 = rectangleSlope[2] * x2 + rectangleExcursion[3][0];
	triangleExcursion[3][0] = y1 - x1*triangleSlope[2];

	y1 = 0;
	x1 = (y1 - rectangleExcursion[3][3]) / rectangleSlope[3];
	x2 = AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[6];
	y2 = x2 * rectangleSlope[3] + rectangleExcursion[3][2];
	triangleExcursion[3][1] = y1 - triangleSlope[3] * x1;

	return 1;
}

int AVMFusing::coordinateCalculate()
{
	float distance, theta, squareCosCita;
	for (int col = rectangleRange[0][0][0]; col < rectangleRange[0][0][1]; col++)
	{
		for (int row = rectangleRange[0][1][0]; row < rectangleRange[0][1][1]; row++)
		{
			if ((row < col*rectangleSlope[0] + rectangleExcursion[0][0]) && (row < col*rectangleSlope[1] + rectangleExcursion[0][2]))
			{
				if (row > col*triangleSlope[0] + triangleExcursion[0][0] && col <= triangleRange[0][0])
				{
					if (col == imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[0])
						theta = CV_PI / 2;
					else
						theta = atan(float(imageHomographyHeight - 1 - row) / float(imageHomographyHeight - 1 - AVMParamters.fusingState.translationCoordinate[0] - col));

					squareCosCita = sin(theta)*sin(theta);
					triangleWeight[0][pointNumber[0][3]] = squareCosCita;
					triangleCoordinate[0][0][pointNumber[0][3]] = row + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.jiontPoint[1];
					triangleCoordinate[0][1][pointNumber[0][3]] = col + AVMParamters.fusingState.translationCoordinate[0] - AVMParamters.fusingState.jiontPoint[0];
					triangleCoordinate[0][2][pointNumber[0][3]] = row;
					triangleCoordinate[0][3][pointNumber[0][3]] = col;
					pointNumber[0][3]++;

				}
				else if ((row > col*rectangleSlope[0] + rectangleExcursion[0][1]) && (col < triangleRange[0][0]))
				{
					distance = -(rectangleSlope[0] * col + rectangleExcursion[0][1] - row) / sqrt(1 + rectangleSlope[0] * rectangleSlope[0]);
					theta = (CV_PI / 200.0)*distance;
					squareCosCita = std::cos(theta) *  std::cos(theta);

					rectangleWeight[0][pointNumber[0][1]] = squareCosCita;
					rectangleCoordinate[0][0][pointNumber[0][1]] = row + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.jiontPoint[1];
					rectangleCoordinate[0][1][pointNumber[0][1]] = col + AVMParamters.fusingState.translationCoordinate[0] - AVMParamters.fusingState.jiontPoint[0];
					rectangleCoordinate[0][2][pointNumber[0][1]] = row;
					rectangleCoordinate[0][3][pointNumber[0][1]] = col;
					pointNumber[0][1]++;
				}
				else if (row > col*triangleSlope[1] + triangleExcursion[0][1] && col >= triangleRange[0][1])
				{
					if (col == AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.translationCoordinate[0])
						theta = CV_PI / 2;
					else
						theta = atan(float(imageHomographyHeight - 1 - row) / float(col - AVMParamters.fusingState.translationCoordinate[4] + AVMParamters.fusingState.translationCoordinate[0]));

					squareCosCita = sin(theta)*sin(theta);
					triangleWeight[1][pointNumber[0][4]] = squareCosCita;
					triangleCoordinate[1][0][pointNumber[0][4]] = row + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.jiontPoint[1];
					triangleCoordinate[1][1][pointNumber[0][4]] = col + AVMParamters.fusingState.translationCoordinate[0] - AVMParamters.fusingState.jiontPoint[0];
					triangleCoordinate[1][2][pointNumber[0][4]] = row;
					triangleCoordinate[1][3][pointNumber[0][4]] = col;
					pointNumber[0][4]++;

				}
				else if ((row > col*rectangleSlope[1] + rectangleExcursion[0][3]) && (col > triangleRange[0][1]))
				{

					distance = -(rectangleSlope[1] * col + rectangleExcursion[0][3] - row) / sqrt(1 + rectangleSlope[1] * rectangleSlope[1]);
					theta = (CV_PI / 200.0)*distance;
					squareCosCita = std::cos(theta) *  std::cos(theta);

					rectangleWeight[1][pointNumber[0][2]] = squareCosCita;
					rectangleCoordinate[1][0][pointNumber[0][2]] = row + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.jiontPoint[1];
					rectangleCoordinate[1][1][pointNumber[0][2]] = col + AVMParamters.fusingState.translationCoordinate[0] - AVMParamters.fusingState.jiontPoint[0];
					rectangleCoordinate[1][2][pointNumber[0][2]] = row;
					rectangleCoordinate[1][3][pointNumber[0][2]] = col;
					pointNumber[0][2]++;

					//imageFused.at<Vec3b>(row - rectangleRange[0][1][0], col - rectangleRange[0][0][0]) = imageFront.at<Vec3b>(row, col);
				}
				else
				{
					fundamentalCoordinate[0][0][pointNumber[0][0]] = row + AVMParamters.fusingState.translationCoordinate[1] - AVMParamters.fusingState.jiontPoint[1];
					fundamentalCoordinate[0][1][pointNumber[0][0]] = col + AVMParamters.fusingState.translationCoordinate[0] - AVMParamters.fusingState.jiontPoint[0];
					fundamentalCoordinate[0][2][pointNumber[0][0]] = row;
					fundamentalCoordinate[0][3][pointNumber[0][0]] = col;
					pointNumber[0][0]++;
					//imageFused.at<Vec3b>(row- rectangleRange[0][1][0], col- rectangleRange[0][0][0]) = imageFront.at<Vec3b>(row, col);

				}
			}
		}
	}

	for (int col = rectangleRange[1][0][0]; col < rectangleRange[1][0][1]; col++)
	{
		for (int row = rectangleRange[1][1][0]; row < rectangleRange[1][1][1]; row++)
		{
			if ((row > col*rectangleSlope[0] + rectangleExcursion[1][1]) && (row < col*rectangleSlope[2] + rectangleExcursion[1][2]))
			{
				//imageLeft.at<Vec3b>(row, col) = { 255, 255, 255 };
				if (row > col*triangleSlope[0] + triangleExcursion[1][0] && row <= triangleRange[1][0])
				{
					triangleCoordinate[0][4][pointNumber[1][3]] = row;
					triangleCoordinate[0][5][pointNumber[1][3]] = col;
					pointNumber[1][3]++;
					//imageLeft.at<Vec3b>(row, col) = { 255, 0, 0 };
				}
				else if ((row < col*rectangleSlope[0] + rectangleExcursion[1][0]) && (row < triangleRange[1][0]))
				{
					rectangleCoordinate[0][4][pointNumber[1][1]] = row;
					rectangleCoordinate[0][5][pointNumber[1][1]] = col;
					pointNumber[1][1]++;
					//imageLeft.at<Vec3b>(row, col) = { 255, 255, 0 };
				}
				else if (row < col*triangleSlope[2] + triangleExcursion[1][1] && row >= triangleRange[1][1])
				{
					if (row == AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[3])
						theta = CV_PI / 2;
					else
						theta = atan(float(imageHomographyHeight - 1 - col) / float(row - AVMParamters.fusingState.translationCoordinate[7] + AVMParamters.fusingState.translationCoordinate[3]));

					squareCosCita = sin(theta)*sin(theta);
					triangleWeight[2][pointNumber[1][4]] = squareCosCita;
					triangleCoordinate[2][0][pointNumber[1][4]] = row + AVMParamters.fusingState.translationCoordinate[3] - AVMParamters.fusingState.jiontPoint[1];
					triangleCoordinate[2][1][pointNumber[1][4]] = col + AVMParamters.fusingState.translationCoordinate[2] - AVMParamters.fusingState.jiontPoint[0];
					triangleCoordinate[2][2][pointNumber[1][4]] = row;
					triangleCoordinate[2][3][pointNumber[1][4]] = col;
					pointNumber[1][4]++;
					//imageLeft.at<Vec3b>(row, col) = { 0, 255, 0 };
				}
				else if ((row > col*rectangleSlope[2] + rectangleExcursion[1][3]) && (row > triangleRange[1][1]))
				{
					distance = -(rectangleSlope[2] * col + rectangleExcursion[1][3] - row) / sqrt(1 + rectangleSlope[2] * rectangleSlope[2]);
					theta = (CV_PI / 200.0)*distance;
					squareCosCita = std::cos(theta) *  std::cos(theta);

					rectangleWeight[2][pointNumber[1][2]] = squareCosCita;
					rectangleCoordinate[2][0][pointNumber[1][2]] = row + AVMParamters.fusingState.translationCoordinate[3] - AVMParamters.fusingState.jiontPoint[1];
					rectangleCoordinate[2][1][pointNumber[1][2]] = col + AVMParamters.fusingState.translationCoordinate[2] - AVMParamters.fusingState.jiontPoint[0];
					rectangleCoordinate[2][2][pointNumber[1][2]] = row;
					rectangleCoordinate[2][3][pointNumber[1][2]] = col;
					pointNumber[1][2]++;
					//imageLeft.at<Vec3b>(row, col) = { 0, 255, 255 };
				}
				else
				{
					fundamentalCoordinate[1][0][pointNumber[1][0]] = row + AVMParamters.fusingState.translationCoordinate[3] - AVMParamters.fusingState.jiontPoint[1];
					fundamentalCoordinate[1][1][pointNumber[1][0]] = col + AVMParamters.fusingState.translationCoordinate[2] - AVMParamters.fusingState.jiontPoint[0];
					//cout << pointNumber[1][0] << "        " << fundamentalCoordinate[1][1][pointNumber[1][0]] << endl;
					fundamentalCoordinate[1][2][pointNumber[1][0]] = row;
					fundamentalCoordinate[1][3][pointNumber[1][0]] = col;
					pointNumber[1][0]++;
					//imageLeft.at<Vec3b>(row, col) = {0, 0, 255};
				}
			}
		}
	}

	for (int col = rectangleRange[2][0][0]; col < rectangleRange[2][0][1]; col++)
	{
		for (int row = rectangleRange[2][1][0]; row < rectangleRange[2][1][1]; row++)
		{
			if ((row > col*rectangleSlope[1] + rectangleExcursion[2][1]) && (row < col*rectangleSlope[3] + rectangleExcursion[2][2]))
			{
				//imageRight.at<Vec3b>(row, col) = { 255, 255, 255 };

				if (row > col*triangleSlope[1] + triangleExcursion[2][0] && row <= triangleRange[2][0])
				{
					triangleCoordinate[1][4][pointNumber[2][3]] = row;
					triangleCoordinate[1][5][pointNumber[2][3]] = col;
					pointNumber[2][3]++;
					//imageRight.at<Vec3b>(row, col) = { 255, 0, 0 };
				}
				else if ((row < col*rectangleSlope[1] + rectangleExcursion[2][0]) && (row < triangleRange[2][0]))
				{
					rectangleCoordinate[1][4][pointNumber[2][1]] = row;
					rectangleCoordinate[1][5][pointNumber[2][1]] = col;
					pointNumber[2][1]++;

					//imageRight.at<Vec3b>(row, col) = { 255, 255, 0 };
				}
				else if (row < col*triangleSlope[3] + triangleExcursion[2][1] && row >= triangleRange[2][1])
				{
					if (row == AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.translationCoordinate[5])
						theta = CV_PI / 2;
					else
						theta = atan(float(col) / float(row - AVMParamters.fusingState.translationCoordinate[7] + AVMParamters.fusingState.translationCoordinate[5]));

					squareCosCita = sin(theta)*sin(theta);
					triangleWeight[3][pointNumber[2][4]] = squareCosCita;
					//std:: cout << triangleWeight[3][pointNumber[2][4]] << std::endl;
					triangleCoordinate[3][0][pointNumber[2][4]] = row + AVMParamters.fusingState.translationCoordinate[5] - AVMParamters.fusingState.jiontPoint[1];
					triangleCoordinate[3][1][pointNumber[2][4]] = col + AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.jiontPoint[0];
					triangleCoordinate[3][2][pointNumber[2][4]] = row;
					triangleCoordinate[3][3][pointNumber[2][4]] = col;
					pointNumber[2][4]++;


					//imageRight.at<Vec3b>(row, col) = { 0, 255, 0 };
				}
				else if ((row > col*rectangleSlope[3] + rectangleExcursion[2][3]) && (row > triangleRange[2][1]))
				{
					distance = -(rectangleSlope[3] * col + rectangleExcursion[2][3] - row) / sqrt(1 + rectangleSlope[3] * rectangleSlope[3]);
					theta = (CV_PI / 200.0)*distance;
					squareCosCita = std::cos(theta) *  std::cos(theta);

					rectangleWeight[3][pointNumber[2][2]] = squareCosCita;
					rectangleCoordinate[3][0][pointNumber[2][2]] = row + AVMParamters.fusingState.translationCoordinate[5] - AVMParamters.fusingState.jiontPoint[1];
					rectangleCoordinate[3][1][pointNumber[2][2]] = col + AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.jiontPoint[0];
					rectangleCoordinate[3][2][pointNumber[2][2]] = row;
					rectangleCoordinate[3][3][pointNumber[2][2]] = col;
					pointNumber[2][2]++;
					//imageRight.at<Vec3b>(row, col) = { 0, 255, 255 };
				}
				else
				{
					fundamentalCoordinate[2][0][pointNumber[2][0]] = row + AVMParamters.fusingState.translationCoordinate[5] - AVMParamters.fusingState.jiontPoint[1];
					fundamentalCoordinate[2][1][pointNumber[2][0]] = col + AVMParamters.fusingState.translationCoordinate[4] - AVMParamters.fusingState.jiontPoint[0];
					fundamentalCoordinate[2][2][pointNumber[2][0]] = row;
					fundamentalCoordinate[2][3][pointNumber[2][0]] = col;
					pointNumber[2][0]++;
					//imageRight.at<Vec3b>(row, col) = { 0, 0, 255 };
				}
			}
		}
	}

	for (int col = rectangleRange[3][0][0]; col < rectangleRange[3][0][1]; col++)
	{
		for (int row = rectangleRange[3][1][0]; row < rectangleRange[3][1][1]; row++)
		{
			if ((row > col*rectangleSlope[2] + rectangleExcursion[3][1]) && (row > col*rectangleSlope[3] + rectangleExcursion[3][3]))
			{
				//imageBack.at<Vec3b>(row, col) = { 255, 255, 255 };
				if (row < col*triangleSlope[2] + triangleExcursion[3][0] && col <= triangleRange[3][0])
				{
					triangleCoordinate[2][4][pointNumber[3][3]] = row;
					triangleCoordinate[2][5][pointNumber[3][3]] = col;
					pointNumber[3][3]++;
					//imageBack.at<Vec3b>(row, col) = { 255, 0, 0 };
				}
				else if ((row < col*rectangleSlope[2] + rectangleExcursion[3][0]) && (col < triangleRange[3][0]))
				{
					rectangleCoordinate[2][4][pointNumber[3][1]] = row;
					rectangleCoordinate[2][5][pointNumber[3][1]] = col;
					pointNumber[3][1]++;
					//imageBack.at<Vec3b>(row, col) = { 255, 255, 0 };
				}
				else if (row < col*triangleSlope[3] + triangleExcursion[3][1] && col >= triangleRange[3][1])
				{
					triangleCoordinate[3][4][pointNumber[3][4]] = row;
					triangleCoordinate[3][5][pointNumber[3][4]] = col;
					pointNumber[3][4]++;
					//imageBack.at<Vec3b>(row, col) = { 0, 255, 0 };
				}
				else if ((row < col*rectangleSlope[3] + rectangleExcursion[3][2]) && (col > triangleRange[3][1]))
				{
					rectangleCoordinate[3][4][pointNumber[3][2]] = row;
					rectangleCoordinate[3][5][pointNumber[3][2]] = col;
					pointNumber[3][2]++;
					//imageBack.at<Vec3b>(row, col) = { 0, 255, 255 };
				}
				else
				{
					fundamentalCoordinate[3][0][pointNumber[3][0]] = row + AVMParamters.fusingState.translationCoordinate[7] - AVMParamters.fusingState.jiontPoint[1];
					fundamentalCoordinate[3][1][pointNumber[3][0]] = col + AVMParamters.fusingState.translationCoordinate[6] - AVMParamters.fusingState.jiontPoint[0];
					fundamentalCoordinate[3][2][pointNumber[3][0]] = row;
					fundamentalCoordinate[3][3][pointNumber[3][0]] = col;
					pointNumber[3][0]++;
					//imageBack.at<Vec3b>(row, col) = { 0, 0, 255 };
				}
			}
		}
	}
	return 1;
}

int AVMFusing::fundamentalTransform()
{
	for (int i = 0; i < pointNumber[0][0]; i++)
	{
		imageFused.at<Vec3b>(fundamentalCoordinate[0][0][i], fundamentalCoordinate[0][1][i]) = imageTransformation[0]->imageSource.at<Vec3b>(fundamentalCoordinate[0][2][i], fundamentalCoordinate[0][3][i]);
	}
	for (int i = 0; i < pointNumber[1][0]; i++)
	{
		imageFused.at<Vec3b>(fundamentalCoordinate[1][0][i], fundamentalCoordinate[1][1][i]) = imageTransformation[1]->imageSource.at<Vec3b>(fundamentalCoordinate[1][2][i], fundamentalCoordinate[1][3][i]);
	}
	for (int i = 0; i < pointNumber[2][0]; i++)
	{
		imageFused.at<Vec3b>(fundamentalCoordinate[2][0][i], fundamentalCoordinate[2][1][i]) = imageTransformation[2]->imageSource.at<Vec3b>(fundamentalCoordinate[2][2][i], fundamentalCoordinate[2][3][i]);
	}
	for (int i = 0; i < pointNumber[3][0]; i++)
	{
		imageFused.at<Vec3b>(fundamentalCoordinate[3][0][i], fundamentalCoordinate[3][1][i]) = imageTransformation[3]->imageSource.at<Vec3b>(fundamentalCoordinate[3][2][i], fundamentalCoordinate[3][3][i]);
	}

	return 1;
}

int AVMFusing::rectangleTransform()
{
	for (int i = 0; i < pointNumber[0][1]; i++)
	{
		imageFused.at<Vec3b>(rectangleCoordinate[0][0][i], rectangleCoordinate[0][1][i]) = rectangleWeight[0][i] * imageTransformation[0]->imageSource.at<Vec3b>(rectangleCoordinate[0][2][i], rectangleCoordinate[0][3][i]) + (1.0 - rectangleWeight[0][i])*imageTransformation[1]->imageSource.at<Vec3b>(rectangleCoordinate[0][4][i], rectangleCoordinate[0][5][i]);
	}

	for (int i = 0; i < pointNumber[2][1]; i++)
	{
		imageFused.at<Vec3b>(rectangleCoordinate[1][0][i], rectangleCoordinate[1][1][i]) = rectangleWeight[1][i] * imageTransformation[0]->imageSource.at<Vec3b>(rectangleCoordinate[1][2][i], rectangleCoordinate[1][3][i]) + (1.0 - rectangleWeight[1][i])*imageTransformation[2]->imageSource.at<Vec3b>(rectangleCoordinate[1][4][i], rectangleCoordinate[1][5][i]);
	}

	for (int i = 0; i < pointNumber[1][2]; i++)
	{
		imageFused.at<Vec3b>(rectangleCoordinate[2][0][i], rectangleCoordinate[2][1][i]) = rectangleWeight[2][i] * imageTransformation[1]->imageSource.at<Vec3b>(rectangleCoordinate[2][2][i], rectangleCoordinate[2][3][i]) + (1.0 - rectangleWeight[2][i])*imageTransformation[3]->imageSource.at<Vec3b>(rectangleCoordinate[2][4][i], rectangleCoordinate[2][5][i]);
	}

	for (int i = 0; i < pointNumber[3][2]; i++)
	{
		imageFused.at<Vec3b>(rectangleCoordinate[3][0][i], rectangleCoordinate[3][1][i]) = rectangleWeight[3][i] * imageTransformation[2]->imageSource.at<Vec3b>(rectangleCoordinate[3][2][i], rectangleCoordinate[3][3][i]) + (1.0 - rectangleWeight[3][i])*imageTransformation[3]->imageSource.at<Vec3b>(rectangleCoordinate[3][4][i], rectangleCoordinate[3][5][i]);
	}
	return 1;
}

int AVMFusing::triangleTransform()
{
	for (int i = 0; i < pointNumber[0][3]; i++)
	{
		imageFused.at<Vec3b>(triangleCoordinate[0][0][i], triangleCoordinate[0][1][i]) = triangleWeight[0][i] * imageTransformation[0]->imageSource.at<Vec3b>(triangleCoordinate[0][2][i], triangleCoordinate[0][3][i]) + (1.0 - triangleWeight[0][i])*imageTransformation[1]->imageSource.at<Vec3b>(triangleCoordinate[0][4][i], triangleCoordinate[0][5][i]);
	}

	for (int i = 0; i < pointNumber[2][3]; i++)
	{
		imageFused.at<Vec3b>(triangleCoordinate[1][0][i], triangleCoordinate[1][1][i]) = triangleWeight[1][i] * imageTransformation[0]->imageSource.at<Vec3b>(triangleCoordinate[1][2][i], triangleCoordinate[1][3][i]) + (1.0 - triangleWeight[1][i])*imageTransformation[2]->imageSource.at<Vec3b>(triangleCoordinate[1][4][i], triangleCoordinate[1][5][i]);
	}

	for (int i = 0; i < pointNumber[1][4]; i++)
	{
		imageFused.at<Vec3b>(triangleCoordinate[2][0][i], triangleCoordinate[2][1][i]) = triangleWeight[2][i] * imageTransformation[1]->imageSource.at<Vec3b>(triangleCoordinate[2][2][i], triangleCoordinate[2][3][i]) + (1.0 - triangleWeight[2][i])*imageTransformation[3]->imageSource.at<Vec3b>(triangleCoordinate[2][4][i], triangleCoordinate[2][5][i]);
	}

	for (int i = 0; i < pointNumber[3][4]; i++)
	{
		imageFused.at<Vec3b>(triangleCoordinate[3][0][i], triangleCoordinate[3][1][i]) = triangleWeight[3][i] * imageTransformation[2]->imageSource.at<Vec3b>(triangleCoordinate[3][2][i], triangleCoordinate[3][3][i]) + (1.0 - triangleWeight[3][i])*imageTransformation[3]->imageSource.at<Vec3b>(triangleCoordinate[3][4][i], triangleCoordinate[3][5][i]);
	}
	return 1;
}

int AVMFusing::coordinateTransform()
{
	int number, number1;
	for (int i = 0; i < pointNumber[0][0]; i++)
	{
		number = fundamentalCoordinate[0][2][i] * imageHomographyWidth + fundamentalCoordinate[0][3][i];
		fundamentalCoordinate[0][2][i] = transformCoordinate[0][0][number];
		fundamentalCoordinate[0][3][i] = transformCoordinate[0][1][number];
	}

	for (int i = 0; i < pointNumber[1][0]; i++)
	{
		number = fundamentalCoordinate[1][2][i] * imageHomographyHeight + fundamentalCoordinate[1][3][i];
		fundamentalCoordinate[1][2][i] = transformCoordinate[1][0][number];
		fundamentalCoordinate[1][3][i] = transformCoordinate[1][1][number];
	}

	for (int i = 0; i < pointNumber[2][0]; i++)
	{
		number = fundamentalCoordinate[2][2][i] * imageHomographyHeight + fundamentalCoordinate[2][3][i];
		
		fundamentalCoordinate[2][2][i] = transformCoordinate[2][0][number];
		fundamentalCoordinate[2][3][i] = transformCoordinate[2][1][number];
	}

	for (int i = 0; i < pointNumber[3][0]; i++)
	{
		number = fundamentalCoordinate[3][2][i] * imageHomographyWidth + fundamentalCoordinate[3][3][i];
		fundamentalCoordinate[3][2][i] = transformCoordinate[3][0][number];
		fundamentalCoordinate[3][3][i] = transformCoordinate[3][1][number];
	}

	for (int i = 0; i < pointNumber[0][1]; i++)
	{
		number = rectangleCoordinate[0][2][i] * imageHomographyWidth + rectangleCoordinate[0][3][i];
		number1 = rectangleCoordinate[0][4][i] * imageHomographyHeight + rectangleCoordinate[0][5][i];
		rectangleCoordinate[0][2][i] = transformCoordinate[0][0][number];
		rectangleCoordinate[0][3][i] = transformCoordinate[0][1][number];
		rectangleCoordinate[0][4][i] = transformCoordinate[1][0][number1];
		rectangleCoordinate[0][5][i] = transformCoordinate[1][1][number1];
	}

	for (int i = 0; i < pointNumber[2][1]; i++)
	{
		number = rectangleCoordinate[1][2][i] * imageHomographyWidth + rectangleCoordinate[1][3][i];
		number1 = rectangleCoordinate[1][4][i] * imageHomographyHeight + rectangleCoordinate[1][5][i];
		rectangleCoordinate[1][2][i] = transformCoordinate[0][0][number];
		rectangleCoordinate[1][3][i] = transformCoordinate[0][1][number];
		rectangleCoordinate[1][4][i] = transformCoordinate[2][0][number1];
		rectangleCoordinate[1][5][i] = transformCoordinate[2][1][number1];
	}

	for (int i = 0; i < pointNumber[1][2]; i++)
	{
		number = rectangleCoordinate[2][2][i] * imageHomographyHeight + rectangleCoordinate[2][3][i];
		number1 = rectangleCoordinate[2][4][i] * imageHomographyWidth + rectangleCoordinate[2][5][i];
		rectangleCoordinate[2][2][i] = transformCoordinate[1][0][number];
		rectangleCoordinate[2][3][i] = transformCoordinate[1][1][number];
		rectangleCoordinate[2][4][i] = transformCoordinate[3][0][number1];
		rectangleCoordinate[2][5][i] = transformCoordinate[3][1][number1];
	}

	for (int i = 0; i < pointNumber[3][2]; i++)
	{
		number = rectangleCoordinate[3][2][i] * imageHomographyHeight + rectangleCoordinate[3][3][i];
		number1 = rectangleCoordinate[3][4][i] * imageHomographyWidth + rectangleCoordinate[3][5][i];
		rectangleCoordinate[3][2][i] = transformCoordinate[2][0][number];
		rectangleCoordinate[3][3][i] = transformCoordinate[2][1][number];
		rectangleCoordinate[3][4][i] = transformCoordinate[3][0][number1];
		rectangleCoordinate[3][5][i] = transformCoordinate[3][1][number1];
	}

	for (int i = 0; i < pointNumber[0][3]; i++)
	{
		number = triangleCoordinate[0][2][i] * imageHomographyWidth + triangleCoordinate[0][3][i];
		number1 = triangleCoordinate[0][4][i] * imageHomographyHeight + triangleCoordinate[0][5][i];
		triangleCoordinate[0][2][i] = transformCoordinate[0][0][number];
		triangleCoordinate[0][3][i] = transformCoordinate[0][1][number];
		triangleCoordinate[0][4][i] = transformCoordinate[1][0][number1];
		triangleCoordinate[0][5][i] = transformCoordinate[1][1][number1];
	}

	for (int i = 0; i < pointNumber[2][3]; i++)
	{
		number = triangleCoordinate[1][2][i] * imageHomographyWidth + triangleCoordinate[1][3][i];
		number1 = triangleCoordinate[1][4][i] * imageHomographyHeight + triangleCoordinate[1][5][i];
		triangleCoordinate[1][2][i] = transformCoordinate[0][0][number];
		triangleCoordinate[1][3][i] = transformCoordinate[0][1][number];
		triangleCoordinate[1][4][i] = transformCoordinate[2][0][number1];
		triangleCoordinate[1][5][i] = transformCoordinate[2][1][number1];
	}

	for (int i = 0; i < pointNumber[1][4]; i++)
	{
		number = triangleCoordinate[2][2][i] * imageHomographyHeight + triangleCoordinate[2][3][i];
		number1 = triangleCoordinate[2][4][i] * imageHomographyWidth + triangleCoordinate[2][5][i];
		triangleCoordinate[2][2][i] = transformCoordinate[1][0][number];
		triangleCoordinate[2][3][i] = transformCoordinate[1][1][number];
		triangleCoordinate[2][4][i] = transformCoordinate[3][0][number1];
		triangleCoordinate[2][5][i] = transformCoordinate[3][1][number1];
	}

	for (int i = 0; i < pointNumber[3][4]; i++)
	{
		number = triangleCoordinate[3][2][i] * imageHomographyHeight + triangleCoordinate[3][3][i];
		number1 = triangleCoordinate[3][4][i] * imageHomographyWidth + triangleCoordinate[3][5][i];
		triangleCoordinate[3][2][i] = transformCoordinate[2][0][number];
		triangleCoordinate[3][3][i] = transformCoordinate[2][1][number];
		triangleCoordinate[3][4][i] = transformCoordinate[3][0][number1];
		triangleCoordinate[3][5][i] = transformCoordinate[3][1][number1];
	}

	return 1;
}

int AVMFusing::getBGRFiles()
{
	FILE *FundamentalSrcRow = fopen(BGRFundamentalPath[0].data(), "wb");
	FILE *FundamentalSrcCol = fopen(BGRFundamentalPath[1].data(), "wb");
	FILE *FundamentalDstRow = fopen(BGRFundamentalPath[2].data(), "wb");
	FILE *FundamentalDstCol = fopen(BGRFundamentalPath[3].data(), "wb");

	for (int i = 0; i < pointNumber[0][0]; i++)
	{
		fwrite(&fundamentalCoordinate[0][0][i], sizeof(short), 1, FundamentalDstRow);
		fwrite(&fundamentalCoordinate[0][1][i], sizeof(short), 1, FundamentalDstCol);
		fwrite(&fundamentalCoordinate[0][2][i], sizeof(short), 1, FundamentalSrcRow);
		fwrite(&fundamentalCoordinate[0][3][i], sizeof(short), 1, FundamentalSrcCol);
	}

	for (int i = 0; i < pointNumber[1][0]; i++)
	{
		fwrite(&fundamentalCoordinate[1][0][i], sizeof(short), 1, FundamentalDstRow);
		fwrite(&fundamentalCoordinate[1][1][i], sizeof(short), 1, FundamentalDstCol);
		fwrite(&fundamentalCoordinate[1][2][i], sizeof(short), 1, FundamentalSrcRow);
		fwrite(&fundamentalCoordinate[1][3][i], sizeof(short), 1, FundamentalSrcCol);
	}

	for (int i = 0; i < pointNumber[2][0]; i++)
	{
		fwrite(&fundamentalCoordinate[2][0][i], sizeof(short), 1, FundamentalDstRow);
		fwrite(&fundamentalCoordinate[2][1][i], sizeof(short), 1, FundamentalDstCol);
		fwrite(&fundamentalCoordinate[2][2][i], sizeof(short), 1, FundamentalSrcRow);
		fwrite(&fundamentalCoordinate[2][3][i], sizeof(short), 1, FundamentalSrcCol);
	}

	for (int i = 0; i < pointNumber[3][0]; i++)
	{
		fwrite(&fundamentalCoordinate[3][0][i], sizeof(short), 1, FundamentalDstRow);
		fwrite(&fundamentalCoordinate[3][1][i], sizeof(short), 1, FundamentalDstCol);
		fwrite(&fundamentalCoordinate[3][2][i], sizeof(short), 1, FundamentalSrcRow);
		fwrite(&fundamentalCoordinate[3][3][i], sizeof(short), 1, FundamentalSrcCol);
	}

	fclose(FundamentalSrcRow);
	fclose(FundamentalSrcCol);
	fclose(FundamentalDstRow);
	fclose(FundamentalDstCol);

	int number1 = 0;
	FILE *RectangleDstRow = fopen(BGRRectanglePath[0].data(), "wb");
	FILE *RectangleDstCol = fopen(BGRRectanglePath[1].data(), "wb");
	FILE *RectangleSrc1Row = fopen(BGRRectanglePath[2].data(), "wb");
	FILE *RectangleSrc1Col = fopen(BGRRectanglePath[3].data(), "wb");
	FILE *RectangleSrc2Row = fopen(BGRRectanglePath[4].data(), "wb");
	FILE *RectangleSrc2Col = fopen(BGRRectanglePath[5].data(), "wb");
	FILE *RectangleDistance = fopen(BGRRectanglePath[6].data(), "wb");

	for (int i = 0; i < pointNumber[0][1]; i++)
	{
		fwrite(&rectangleCoordinate[0][0][i], sizeof(short), 1, RectangleDstRow);
		fwrite(&rectangleCoordinate[0][1][i], sizeof(short), 1, RectangleDstCol);
		fwrite(&rectangleCoordinate[0][2][i], sizeof(short), 1, RectangleSrc1Row);
		fwrite(&rectangleCoordinate[0][3][i], sizeof(short), 1, RectangleSrc1Col);
		fwrite(&rectangleCoordinate[0][4][i], sizeof(short), 1, RectangleSrc2Row);
		fwrite(&rectangleCoordinate[0][5][i], sizeof(short), 1, RectangleSrc2Col);
		fwrite(&rectangleWeight[0][i], sizeof(float), 1, RectangleDistance);
	}

	for (int i = 0; i < pointNumber[2][1]; i++)
	{
		fwrite(&rectangleCoordinate[1][0][i], sizeof(short), 1, RectangleDstRow);
		fwrite(&rectangleCoordinate[1][1][i], sizeof(short), 1, RectangleDstCol);
		fwrite(&rectangleCoordinate[1][2][i], sizeof(short), 1, RectangleSrc1Row);
		fwrite(&rectangleCoordinate[1][3][i], sizeof(short), 1, RectangleSrc1Col);
		fwrite(&rectangleCoordinate[1][4][i], sizeof(short), 1, RectangleSrc2Row);
		fwrite(&rectangleCoordinate[1][5][i], sizeof(short), 1, RectangleSrc2Col);
		fwrite(&rectangleWeight[1][i], sizeof(float), 1, RectangleDistance);
	}

	for (int i = 0; i < pointNumber[1][2]; i++)
	{
		fwrite(&rectangleCoordinate[2][0][i], sizeof(short), 1, RectangleDstRow);
		fwrite(&rectangleCoordinate[2][1][i], sizeof(short), 1, RectangleDstCol);
		fwrite(&rectangleCoordinate[2][2][i], sizeof(short), 1, RectangleSrc1Row);
		fwrite(&rectangleCoordinate[2][3][i], sizeof(short), 1, RectangleSrc1Col);
		fwrite(&rectangleCoordinate[2][4][i], sizeof(short), 1, RectangleSrc2Row);
		fwrite(&rectangleCoordinate[2][5][i], sizeof(short), 1, RectangleSrc2Col);
		fwrite(&rectangleWeight[2][i], sizeof(float), 1, RectangleDistance);

	}

	for (int i = 0; i < pointNumber[3][2]; i++)
	{
		fwrite(&rectangleCoordinate[3][0][i], sizeof(short), 1, RectangleDstRow);
		fwrite(&rectangleCoordinate[3][1][i], sizeof(short), 1, RectangleDstCol);
		fwrite(&rectangleCoordinate[3][2][i], sizeof(short), 1, RectangleSrc1Row);
		fwrite(&rectangleCoordinate[3][3][i], sizeof(short), 1, RectangleSrc1Col);
		fwrite(&rectangleCoordinate[3][4][i], sizeof(short), 1, RectangleSrc2Row);
		fwrite(&rectangleCoordinate[3][5][i], sizeof(short), 1, RectangleSrc2Col);
		fwrite(&rectangleWeight[3][i], sizeof(float), 1, RectangleDistance);
	}

	fclose(RectangleDstRow);
	fclose(RectangleDstCol);
	fclose(RectangleSrc1Row);
	fclose(RectangleSrc1Col);
	fclose(RectangleSrc2Row);
	fclose(RectangleSrc2Col);
	fclose(RectangleDistance);


	FILE *TriangleDstRow = fopen(BGRTrianglePath[0].data(), "wb");
	FILE *TriangleDstCol = fopen(BGRTrianglePath[1].data(), "wb");
	FILE *TriangleSrc1Row = fopen(BGRTrianglePath[2].data(), "wb");
	FILE *TriangleSrc1Col = fopen(BGRTrianglePath[3].data(), "wb");
	FILE *TriangleSrc2Row = fopen(BGRTrianglePath[4].data(), "wb");
	FILE *TriangleSrc2Col = fopen(BGRTrianglePath[5].data(), "wb");
	FILE *TriangleDistance = fopen(BGRTrianglePath[6].data(), "wb");

	for (int i = 0; i < pointNumber[0][3]; i++)
	{
		fwrite(&triangleCoordinate[0][0][i], sizeof(short), 1, TriangleDstRow);
		fwrite(&triangleCoordinate[0][1][i], sizeof(short), 1, TriangleDstCol);
		fwrite(&triangleCoordinate[0][2][i], sizeof(short), 1, TriangleSrc1Row);
		fwrite(&triangleCoordinate[0][3][i], sizeof(short), 1, TriangleSrc1Col);
		fwrite(&triangleCoordinate[0][4][i], sizeof(short), 1, TriangleSrc2Row);
		fwrite(&triangleCoordinate[0][5][i], sizeof(short), 1, TriangleSrc2Col);
		fwrite(&triangleWeight[0][i], sizeof(float), 1, TriangleDistance);
	}

	for (int i = 0; i < pointNumber[2][3]; i++)
	{
		fwrite(&triangleCoordinate[1][0][i], sizeof(short), 1, TriangleDstRow);
		fwrite(&triangleCoordinate[1][1][i], sizeof(short), 1, TriangleDstCol);
		fwrite(&triangleCoordinate[1][2][i], sizeof(short), 1, TriangleSrc1Row);
		fwrite(&triangleCoordinate[1][3][i], sizeof(short), 1, TriangleSrc1Col);
		fwrite(&triangleCoordinate[1][4][i], sizeof(short), 1, TriangleSrc2Row);
		fwrite(&triangleCoordinate[1][5][i], sizeof(short), 1, TriangleSrc2Col);
		fwrite(&triangleWeight[1][i], sizeof(float), 1, TriangleDistance);
	}

	for (int i = 0; i < pointNumber[1][4]; i++)
	{
		fwrite(&triangleCoordinate[2][0][i], sizeof(short), 1, TriangleDstRow);
		fwrite(&triangleCoordinate[2][1][i], sizeof(short), 1, TriangleDstCol);
		fwrite(&triangleCoordinate[2][2][i], sizeof(short), 1, TriangleSrc1Row);
		fwrite(&triangleCoordinate[2][3][i], sizeof(short), 1, TriangleSrc1Col);
		fwrite(&triangleCoordinate[2][4][i], sizeof(short), 1, TriangleSrc2Row);
		fwrite(&triangleCoordinate[2][5][i], sizeof(short), 1, TriangleSrc2Col);
		fwrite(&triangleWeight[2][i], sizeof(float), 1, TriangleDistance);
	}

	for (int i = 0; i < pointNumber[3][4]; i++)
	{
		fwrite(&triangleCoordinate[3][0][i], sizeof(short), 1, TriangleDstRow);
		fwrite(&triangleCoordinate[3][1][i], sizeof(short), 1, TriangleDstCol);
		fwrite(&triangleCoordinate[3][2][i], sizeof(short), 1, TriangleSrc1Row);
		fwrite(&triangleCoordinate[3][3][i], sizeof(short), 1, TriangleSrc1Col);
		fwrite(&triangleCoordinate[3][4][i], sizeof(short), 1, TriangleSrc2Row);
		fwrite(&triangleCoordinate[3][5][i], sizeof(short), 1, TriangleSrc2Col);
		fwrite(&triangleWeight[3][i], sizeof(float), 1, TriangleDistance);
	}

	fclose(TriangleDstRow);
	fclose(TriangleDstCol);
	fclose(TriangleSrc1Row);
	fclose(TriangleSrc1Col);
	fclose(TriangleSrc2Row);
	fclose(TriangleSrc2Col);
	fclose(TriangleDistance);

	return 1;
}

int AVMFusing::getYUYVFiles()
{
	int coordinateDst1, coordinateSrc1, coordinateDst2, coordinateSrc2, coordinateSrc3, coordinateSrc4;
	FILE *fileFundamentalDst = fopen(YUYVFundamentalPath[0].data(), "wb");
	FILE *fileFundamentalSrc = fopen(YUYVFundamentalPath[1].data(), "wb");
	for (int i = 0; i < pointNumber[0][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[0][0][i] * targetImageWidth * 2 + fundamentalCoordinate[0][1][i] * 2;
		coordinateSrc1 = fundamentalCoordinate[0][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[0][3][i] * 2;
		if (fundamentalCoordinate[0][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * targetImageWidth * 2 + fundamentalCoordinate[0][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[0][3][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * targetImageWidth * 2 + fundamentalCoordinate[0][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[0][3][i] * 2 + 3;
			}
		}
		else
		{
			if (fundamentalCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * targetImageWidth * 2 + fundamentalCoordinate[0][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[0][3][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * targetImageWidth * 2 + fundamentalCoordinate[0][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[0][3][i] * 2 + 1;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	for (int i = 0; i < pointNumber[1][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[1][0][i] * targetImageWidth * 2 + fundamentalCoordinate[1][1][i] * 2;
		coordinateSrc1 = fundamentalCoordinate[1][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[1][3][i] * 2;
		if (fundamentalCoordinate[1][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * targetImageWidth * 2 + fundamentalCoordinate[1][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[1][3][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * targetImageWidth * 2 + fundamentalCoordinate[1][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[1][3][i] * 2 + 3;
			}
		}
		else
		{
			if (fundamentalCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * targetImageWidth * 2 + fundamentalCoordinate[1][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[1][3][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * targetImageWidth * 2 + fundamentalCoordinate[1][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[1][3][i] * 2 + 1;
			}
		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	for (int i = 0; i < pointNumber[2][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[2][0][i] * targetImageWidth * 2 + fundamentalCoordinate[2][1][i] * 2;
		coordinateSrc1 = fundamentalCoordinate[2][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[2][3][i] * 2;
		if (fundamentalCoordinate[2][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * targetImageWidth * 2 + fundamentalCoordinate[2][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[2][3][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * targetImageWidth * 2 + fundamentalCoordinate[2][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[2][3][i] * 2 + 3;
			}

		}
		else
		{
			if (fundamentalCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * targetImageWidth * 2 + fundamentalCoordinate[2][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[2][3][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * targetImageWidth * 2 + fundamentalCoordinate[2][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[2][3][i] * 2 + 1;
			}
		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	for (int i = 0; i < pointNumber[3][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[3][0][i] * targetImageWidth * 2 + fundamentalCoordinate[3][1][i] * 2;
		coordinateSrc1 = fundamentalCoordinate[3][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[3][3][i] * 2;
		if (fundamentalCoordinate[3][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * targetImageWidth * 2 + fundamentalCoordinate[3][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[3][3][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * targetImageWidth * 2 + fundamentalCoordinate[3][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[3][3][i] * 2 + 3;
			}
		}
		else
		{
			if (fundamentalCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * targetImageWidth * 2 + fundamentalCoordinate[3][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[3][3][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * targetImageWidth * 2 + fundamentalCoordinate[3][1][i] * 2 + 1;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * imageHomographyWidth * 2 + fundamentalCoordinate[3][3][i] * 2 + 1;
			}
		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	fclose(fileFundamentalDst);
	fclose(fileFundamentalSrc);

	FILE *fileRectangleDst = fopen(YUYVRectanglePath[0].data(), "wb");
	FILE *fileRectangleSrc1 = fopen(YUYVRectanglePath[1].data(), "wb");
	FILE *fileRectangleSrc2 = fopen(YUYVRectanglePath[2].data(), "wb");
	FILE *fileRectangleDistance = fopen(YUYVRectanglePath[3].data(), "wb");

	for (int i = 0; i < pointNumber[0][1]; i++)
	{
		coordinateDst1 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2;
		coordinateSrc1 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2;
		coordinateSrc2 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2;

		if (rectangleCoordinate[0][3][i] % 2 == 0 && rectangleCoordinate[0][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 3;
			}
		}

		if (rectangleCoordinate[0][3][i] % 2 == 0 && rectangleCoordinate[0][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[0][3][i] % 2 == 1 && rectangleCoordinate[0][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[0][3][i] % 2 == 1 && rectangleCoordinate[0][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * targetImageWidth * 2 + rectangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[0][i], sizeof(float), 1, fileRectangleDistance);
	}

	for (int i = 0; i < pointNumber[2][1]; i++)
	{
		coordinateDst1 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2;
		coordinateSrc1 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2;
		coordinateSrc2 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2;
		if (rectangleCoordinate[1][3][i] % 2 == 0 && rectangleCoordinate[1][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 3;
			}
		}

		if (rectangleCoordinate[1][3][i] % 2 == 0 && rectangleCoordinate[1][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[1][3][i] % 2 == 1 && rectangleCoordinate[1][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[1][3][i] % 2 == 1 && rectangleCoordinate[1][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * targetImageWidth * 2 + rectangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[1][i], sizeof(float), 1, fileRectangleDistance);
	}

	for (int i = 0; i < pointNumber[1][2]; i++)
	{
		coordinateDst1 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2;
		coordinateSrc1 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2;
		coordinateSrc2 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2;
		if (rectangleCoordinate[2][3][i] % 2 == 0 && rectangleCoordinate[2][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 3;
			}
		}

		if (rectangleCoordinate[2][3][i] % 2 == 0 && rectangleCoordinate[2][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[2][3][i] % 2 == 1 && rectangleCoordinate[2][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[2][3][i] % 2 == 1 && rectangleCoordinate[2][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * targetImageWidth * 2 + rectangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[2][i], sizeof(float), 1, fileRectangleDistance);
	}

	for (int i = 0; i < pointNumber[3][2]; i++)
	{
		coordinateDst1 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2;
		coordinateSrc1 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2;
		coordinateSrc2 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2;
		if (rectangleCoordinate[3][3][i] % 2 == 0 && rectangleCoordinate[3][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 3;
			}
		}

		if (rectangleCoordinate[3][3][i] % 2 == 0 && rectangleCoordinate[3][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 3;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 1;
			}
		}

		if (rectangleCoordinate[3][3][i] % 2 == 1 && rectangleCoordinate[3][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 1;
			}


		}

		if (rectangleCoordinate[3][3][i] % 2 == 1 && rectangleCoordinate[3][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 - 1;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * targetImageWidth * 2 + rectangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * imageHomographyWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[3][i], sizeof(float), 1, fileRectangleDistance);
	}

	fclose(fileRectangleDst);
	fclose(fileRectangleSrc1);
	fclose(fileRectangleSrc2);
	fclose(fileRectangleDistance);

	FILE *fileTriangleDst = fopen(YUYVTrianglePath[0].data(), "wb");
	FILE *fileTriangleSrc1 = fopen(YUYVTrianglePath[1].data(), "wb");
	FILE *fileTriangleSrc2 = fopen(YUYVTrianglePath[2].data(), "wb");
	FILE *fileTriangleDistance = fopen(YUYVTrianglePath[3].data(), "wb");

	for (int i = 0; i < pointNumber[0][3]; i++)
	{
		coordinateDst1 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2;
		coordinateSrc1 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2;
		coordinateSrc2 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2;
		if (triangleCoordinate[0][3][i] % 2 == 0 && triangleCoordinate[0][5][i] % 2 == 0)
		{
			if (triangleCoordinate[0][5][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 + 3;
			}
		}

		if (triangleCoordinate[0][3][i] % 2 == 0 && triangleCoordinate[0][5][i] % 2 == 1)
		{
			if (triangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[0][3][i] % 2 == 1 && triangleCoordinate[0][5][i] % 2 == 1)
		{
			if (triangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[0][3][i] % 2 == 1 && triangleCoordinate[0][5][i] % 2 == 0)
		{
			if (triangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * targetImageWidth * 2 + triangleCoordinate[0][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[0][2][i] * imageHomographyWidth * 2 + triangleCoordinate[0][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[0][4][i] * imageHomographyWidth * 2 + triangleCoordinate[0][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[0][i], sizeof(float), 1, fileTriangleDistance);
	}

	for (int i = 0; i < pointNumber[2][3]; i++)
	{
		coordinateDst1 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2;
		coordinateSrc1 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2;
		coordinateSrc2 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2;
		if (triangleCoordinate[1][3][i] % 2 == 0 && triangleCoordinate[1][5][i] % 2 == 0)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 + 3;
			}
		}

		if (triangleCoordinate[1][3][i] % 2 == 0 && triangleCoordinate[1][5][i] % 2 == 1)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[1][3][i] % 2 == 1 && triangleCoordinate[1][5][i] % 2 == 1)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[1][3][i] % 2 == 1 && triangleCoordinate[1][5][i] % 2 == 0)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * targetImageWidth * 2 + triangleCoordinate[1][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[1][2][i] * imageHomographyWidth * 2 + triangleCoordinate[1][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[1][4][i] * imageHomographyWidth * 2 + triangleCoordinate[1][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[1][i], sizeof(float), 1, fileTriangleDistance);
	}

	for (int i = 0; i < pointNumber[1][4]; i++)
	{
		coordinateDst1 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2;
		coordinateSrc1 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2;
		coordinateSrc2 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2;
		if (triangleCoordinate[2][3][i] % 2 == 0 && triangleCoordinate[2][5][i] % 2 == 0)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 + 3;
			}
		}

		if (triangleCoordinate[2][3][i] % 2 == 0 && triangleCoordinate[2][5][i] % 2 == 1)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[2][3][i] % 2 == 1 && triangleCoordinate[2][5][i] % 2 == 1)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 + 1;
			}

		}

		if (triangleCoordinate[2][3][i] % 2 == 1 && triangleCoordinate[2][5][i] % 2 == 0)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * targetImageWidth * 2 + triangleCoordinate[2][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[2][2][i] * imageHomographyWidth * 2 + triangleCoordinate[2][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[2][4][i] * imageHomographyWidth * 2 + triangleCoordinate[2][5][i] * 2 + 3;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[2][i], sizeof(float), 1, fileTriangleDistance);
	}

	for (int i = 0; i < pointNumber[3][4]; i++)
	{
		coordinateDst1 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2;
		coordinateSrc1 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2;
		coordinateSrc2 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2;
		if (triangleCoordinate[3][3][i] % 2 == 0 && triangleCoordinate[3][5][i] % 2 == 0)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 + 3;
			}
		}

		if (triangleCoordinate[3][3][i] % 2 == 0 && triangleCoordinate[3][5][i] % 2 == 1)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 + 3;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[3][3][i] % 2 == 1 && triangleCoordinate[3][5][i] % 2 == 1)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 - 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 + 1;
			}
		}

		if (triangleCoordinate[3][3][i] % 2 == 1 && triangleCoordinate[3][5][i] % 2 == 0)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 - 1;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 + 1;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * targetImageWidth * 2 + triangleCoordinate[3][1][i] * 2 + 1;
				coordinateSrc3 = triangleCoordinate[3][2][i] * imageHomographyWidth * 2 + triangleCoordinate[3][3][i] * 2 + 1;
				coordinateSrc4 = triangleCoordinate[3][4][i] * imageHomographyWidth * 2 + triangleCoordinate[3][5][i] * 2 + 3;
			}
		}

		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[3][i], sizeof(float), 1, fileTriangleDistance);
	}

	fclose(fileTriangleDst);
	fclose(fileTriangleSrc1);
	fclose(fileTriangleSrc2);
	fclose(fileTriangleDistance);

return 1;
}

int AVMFusing::getUYVYFiles()
{
	int coordinateDst1, coordinateSrc1, coordinateDst2, coordinateSrc2, coordinateSrc3, coordinateSrc4;
	FILE *fileFundamentalDst = fopen(UYVYFundamentalPath[0].data(), "wb");
	FILE *fileFundamentalSrc = fopen(UYVYFundamentalPath[1].data(), "wb");

	for (int i = 0; i < pointNumber[0][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[0][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[0][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X*2;
		coordinateSrc1 = fundamentalCoordinate[0][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[0][3][i] * 2 + 1;
		if (fundamentalCoordinate[0][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[0][3][i] * 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[0][3][i] * 2 + 2;
			}
		}
		else
		{
			if (fundamentalCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[0][3][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[0][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[0][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[0][3][i] * 2 ;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	for (int i = 0; i < pointNumber[1][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[1][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[1][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = fundamentalCoordinate[1][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[1][3][i] * 2 + 1;
		if (fundamentalCoordinate[1][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[1][3][i] * 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[1][3][i] * 2 + 2;
			}
		}
		else
		{
			if (fundamentalCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[1][3][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[1][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[1][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[1][3][i] * 2;
			}
		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	for (int i = 0; i < pointNumber[2][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[2][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[2][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = fundamentalCoordinate[2][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[2][3][i] * 2 + 1;
		if (fundamentalCoordinate[2][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[2][3][i] * 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[2][3][i] * 2 + 2;
			}

		}
		else
		{
			if (fundamentalCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[2][3][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[2][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[2][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[2][3][i] * 2;
			}
		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	for (int i = 0; i < pointNumber[3][0]; i++)
	{
		coordinateDst1 = fundamentalCoordinate[3][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[3][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = fundamentalCoordinate[3][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[3][3][i] * 2 + 1;
		if (fundamentalCoordinate[3][3][i] % 2 == 0)
		{
			if (fundamentalCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[3][3][i] * 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[3][3][i] * 2 + 2;
			}
		}
		else
		{
			if (fundamentalCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[3][3][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = fundamentalCoordinate[3][0][i] * SCREEN_WIDTH * 2 + fundamentalCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc2 = fundamentalCoordinate[3][2][i] * sourceImageWidth * 2 + fundamentalCoordinate[3][3][i] * 2;
			}
		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileFundamentalSrc);
		fwrite(&coordinateDst2, sizeof(int), 1, fileFundamentalDst);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileFundamentalSrc);
	}

	fclose(fileFundamentalDst);
	fclose(fileFundamentalSrc);

	FILE *fileRectangleDst = fopen(UYVYRectanglePath[0].data(), "wb");
	FILE *fileRectangleSrc1 = fopen(UYVYRectanglePath[1].data(), "wb");
	FILE *fileRectangleSrc2 = fopen(UYVYRectanglePath[2].data(), "wb");
	FILE *fileRectangleDistance = fopen(UYVYRectanglePath[3].data(), "wb");

	for (int i = 0; i < pointNumber[0][1]; i++)
	{
		coordinateDst1 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 1;
		coordinateSrc2 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 1;

		if (rectangleCoordinate[0][3][i] % 2 == 0 && rectangleCoordinate[0][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 2;
			}
		}

		if (rectangleCoordinate[0][3][i] % 2 == 0 && rectangleCoordinate[0][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2;
			}
		}

		if (rectangleCoordinate[0][3][i] % 2 == 1 && rectangleCoordinate[0][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2;
			}
		}

		if (rectangleCoordinate[0][3][i] % 2 == 1 && rectangleCoordinate[0][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[0][2][i] * sourceImageWidth * 2 + rectangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[0][4][i] * sourceImageWidth * 2 + rectangleCoordinate[0][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[0][i], sizeof(float), 1, fileRectangleDistance);
	}

	for (int i = 0; i < pointNumber[2][1]; i++)
	{
		coordinateDst1 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 1;
		coordinateSrc2 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 1;
		if (rectangleCoordinate[1][3][i] % 2 == 0 && rectangleCoordinate[1][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 2;
			}
		}

		if (rectangleCoordinate[1][3][i] % 2 == 0 && rectangleCoordinate[1][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2;
			}
		}

		if (rectangleCoordinate[1][3][i] % 2 == 1 && rectangleCoordinate[1][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2;
			}
		}

		if (rectangleCoordinate[1][3][i] % 2 == 1 && rectangleCoordinate[1][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[1][2][i] * sourceImageWidth * 2 + rectangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[1][4][i] * sourceImageWidth * 2 + rectangleCoordinate[1][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[1][i], sizeof(float), 1, fileRectangleDistance);
	}

	for (int i = 0; i < pointNumber[1][2]; i++)
	{
		coordinateDst1 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 1;
		coordinateSrc2 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 1;
		if (rectangleCoordinate[2][3][i] % 2 == 0 && rectangleCoordinate[2][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 2;
			}
		}

		if (rectangleCoordinate[2][3][i] % 2 == 0 && rectangleCoordinate[2][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2;
			}
		}

		if (rectangleCoordinate[2][3][i] % 2 == 1 && rectangleCoordinate[2][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2;
			}
		}

		if (rectangleCoordinate[2][3][i] % 2 == 1 && rectangleCoordinate[2][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[2][2][i] * sourceImageWidth * 2 + rectangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[2][4][i] * sourceImageWidth * 2 + rectangleCoordinate[2][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[2][i], sizeof(float), 1, fileRectangleDistance);
	}

	for (int i = 0; i < pointNumber[3][2]; i++)
	{

		coordinateDst1 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 1;
		coordinateSrc2 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 1;

		if (rectangleCoordinate[3][3][i] % 2 == 0 && rectangleCoordinate[3][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 2;
			}
		}

		if (rectangleCoordinate[3][3][i] % 2 == 0 && rectangleCoordinate[3][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2 + 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2;
			}
		}

		if (rectangleCoordinate[3][3][i] % 2 == 1 && rectangleCoordinate[3][5][i] % 2 == 1)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2;
			}


		}

		if (rectangleCoordinate[3][3][i] % 2 == 1 && rectangleCoordinate[3][5][i] % 2 == 0)
		{
			if (rectangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2 - 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2;
			}
			else
			{
				coordinateDst2 = rectangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + rectangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = rectangleCoordinate[3][2][i] * sourceImageWidth * 2 + rectangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = rectangleCoordinate[3][4][i] * sourceImageWidth * 2 + rectangleCoordinate[3][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileRectangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileRectangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileRectangleSrc2);
		fwrite(&rectangleWeight[3][i], sizeof(float), 1, fileRectangleDistance);
	}

	fclose(fileRectangleDst);
	fclose(fileRectangleSrc1);
	fclose(fileRectangleSrc2);
	fclose(fileRectangleDistance);

	FILE *fileTriangleDst = fopen(UYVYTrianglePath[0].data(), "wb");
	FILE *fileTriangleSrc1 = fopen(UYVYTrianglePath[1].data(), "wb");
	FILE *fileTriangleSrc2 = fopen(UYVYTrianglePath[2].data(), "wb");
	FILE *fileTriangleDistance = fopen(UYVYTrianglePath[3].data(), "wb");

	for (int i = 0; i < pointNumber[0][3]; i++)
	{
		coordinateDst1 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2 + 1;
		coordinateSrc2 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2 + 1;
		if (triangleCoordinate[0][3][i] % 2 == 0 && triangleCoordinate[0][5][i] % 2 == 0)
		{
			if (triangleCoordinate[0][5][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2 + 2;
			}
		}

		if (triangleCoordinate[0][3][i] % 2 == 0 && triangleCoordinate[0][5][i] % 2 == 1)
		{
			if (triangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2;
			}
		}

		if (triangleCoordinate[0][3][i] % 2 == 1 && triangleCoordinate[0][5][i] % 2 == 1)
		{
			if (triangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2;
			}
		}

		if (triangleCoordinate[0][3][i] % 2 == 1 && triangleCoordinate[0][5][i] % 2 == 0)
		{
			if (triangleCoordinate[0][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[0][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[0][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[0][2][i] * sourceImageWidth * 2 + triangleCoordinate[0][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[0][4][i] * sourceImageWidth * 2 + triangleCoordinate[0][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[0][i], sizeof(float), 1, fileTriangleDistance);
	}

	for (int i = 0; i < pointNumber[2][3]; i++)
	{
		coordinateDst1 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2 + 1;
		coordinateSrc2 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2 + 1;
		if (triangleCoordinate[1][3][i] % 2 == 0 && triangleCoordinate[1][5][i] % 2 == 0)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2 + 2;
			}
		}

		if (triangleCoordinate[1][3][i] % 2 == 0 && triangleCoordinate[1][5][i] % 2 == 1)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2;
			}
		}

		if (triangleCoordinate[1][3][i] % 2 == 1 && triangleCoordinate[1][5][i] % 2 == 1)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2;
			}
		}

		if (triangleCoordinate[1][3][i] % 2 == 1 && triangleCoordinate[1][5][i] % 2 == 0)
		{
			if (triangleCoordinate[1][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[1][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[1][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[1][2][i] * sourceImageWidth * 2 + triangleCoordinate[1][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[1][4][i] * sourceImageWidth * 2 + triangleCoordinate[1][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[1][i], sizeof(float), 1, fileTriangleDistance);
	}

	for (int i = 0; i < pointNumber[1][4]; i++)
	{
		coordinateDst1 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2 + 1;
		coordinateSrc2 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2 + 1;
		if (triangleCoordinate[2][3][i] % 2 == 0 && triangleCoordinate[2][5][i] % 2 == 0)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2 + 2;
			}
		}

		if (triangleCoordinate[2][3][i] % 2 == 0 && triangleCoordinate[2][5][i] % 2 == 1)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2;
			}
		}

		if (triangleCoordinate[2][3][i] % 2 == 1 && triangleCoordinate[2][5][i] % 2 == 1)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2;
			}

		}

		if (triangleCoordinate[2][3][i] % 2 == 1 && triangleCoordinate[2][5][i] % 2 == 0)
		{
			if (triangleCoordinate[2][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[2][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[2][1][i] * 2;
				coordinateSrc3 = triangleCoordinate[2][2][i] * sourceImageWidth * 2 + triangleCoordinate[2][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[2][4][i] * sourceImageWidth * 2 + triangleCoordinate[2][5][i] * 2 + 2;
			}

		}
		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[2][i], sizeof(float), 1, fileTriangleDistance);
	}

	for (int i = 0; i < pointNumber[3][4]; i++)
	{
		coordinateDst1 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + 1 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
		coordinateSrc1 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2 + 1;
		coordinateSrc2 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2 + 1;
		if (triangleCoordinate[3][3][i] % 2 == 0 && triangleCoordinate[3][5][i] % 2 == 0)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2 + 2;
			}
		}

		if (triangleCoordinate[3][3][i] % 2 == 0 && triangleCoordinate[3][5][i] % 2 == 1)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2 + 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2;
			}
		}

		if (triangleCoordinate[3][3][i] % 2 == 1 && triangleCoordinate[3][5][i] % 2 == 1)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2 - 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2;
			}
		}

		if (triangleCoordinate[3][3][i] % 2 == 1 && triangleCoordinate[3][5][i] % 2 == 0)
		{
			if (triangleCoordinate[3][1][i] % 2 == 0)
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2 - 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2;
			}
			else
			{
				coordinateDst2 = triangleCoordinate[3][0][i] * SCREEN_WIDTH * 2 + triangleCoordinate[3][1][i] * 2 + IMAGE_POSITION_Y * SCREEN_WIDTH * 2 + IMAGE_POSITION_X * 2;
				coordinateSrc3 = triangleCoordinate[3][2][i] * sourceImageWidth * 2 + triangleCoordinate[3][3][i] * 2;
				coordinateSrc4 = triangleCoordinate[3][4][i] * sourceImageWidth * 2 + triangleCoordinate[3][5][i] * 2 + 2;
			}
		}

		fwrite(&coordinateDst1, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateDst2, sizeof(int), 1, fileTriangleDst);
		fwrite(&coordinateSrc1, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc2, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&coordinateSrc3, sizeof(int), 1, fileTriangleSrc1);
		fwrite(&coordinateSrc4, sizeof(int), 1, fileTriangleSrc2);
		fwrite(&triangleWeight[3][i], sizeof(float), 1, fileTriangleDistance);
	}

	fclose(fileTriangleDst);
	fclose(fileTriangleSrc1);
	fclose(fileTriangleSrc2);
	fclose(fileTriangleDistance);

	return 1;
}

int AVMFusing::getBGRParametersFiles()
{
	FILE *AVMNumbers = fopen(BGRNumberPath.data(), "wb");
	fwrite(&pointNumber[0][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[0][1], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][1], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][2], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][2], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[0][3], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][3], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][4], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][4], sizeof(int), 1, AVMNumbers);
	fclose(AVMNumbers);
	return 1;
}

int AVMFusing::getYUYVParametersFiles()
{
	FILE *AVMNumbers = fopen(YUYVNumberPath.data(), "wb");
	fwrite(&pointNumber[0][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[0][1], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][1], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][2], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][2], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[0][3], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][3], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][4], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][4], sizeof(int), 1, AVMNumbers);
	fclose(AVMNumbers);
	return 1;
}

int AVMFusing::getUYVYParametersFiles()
{
	FILE *AVMNumbers = fopen(UYVYNumberPath.data(), "wb");
	fwrite(&pointNumber[0][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][0], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[0][1], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][1], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][2], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][2], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[0][3], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[2][3], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[1][4], sizeof(int), 1, AVMNumbers);
	fwrite(&pointNumber[3][4], sizeof(int), 1, AVMNumbers);
	fclose(AVMNumbers);
	return 1;
}