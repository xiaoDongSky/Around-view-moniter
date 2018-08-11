#include "xmlFileRead.h"

int xmlFileRead(XmlFileState &xmlFileState,string xmlPath0, string xmlPath1,string xmlPath2, string xmlPath3)
{

	for (int i = 0; i < 4; i++)
	{
		xmlFileState.cameraState[i].intrinsicMatrix.create(3, 3, CV_32FC1);
		xmlFileState.cameraState[i].homographyMatrix.create(3, 3, CV_32FC1);
		//xmlFileState.cameraState[i].affMatrix.create(3, 3, CV_32FC1);
	}

	for (int i = 0; i < 8; i++)
	{
		xmlFileState.fusingState.translationCoordinate[i] = 0;
	}

	FileStorage fileParameter0(xmlPath0, FileStorage::READ);

	if (fileParameter0.isOpened() == 0)
	{
		cout << "Parameter0 open failed" << endl;
		return -1;
	}
	vector<string> xml0Tags = { "camera0", "camera1","camera2","camera3","distortCoffs0","distortCoffs1","distortCoffs2","distortCoffs3" };
	for (int i = 0; i < 4; i++)
	{
		fileParameter0[xml0Tags[i]] >> xmlFileState.cameraState[i].intrinsicMatrix;	
	}
	for (int i = 4; i < 8; i++)
	{
		fileParameter0[xml0Tags[i]] >> xmlFileState.cameraState[i-4].distortCoffs;
	}
	fileParameter0["imageRatio0"] >> xmlFileState.cameraState[0].ratio;
	fileParameter0["imageRatio1"] >> xmlFileState.cameraState[1].ratio;
	fileParameter0["imageRatio2"] >> xmlFileState.cameraState[2].ratio;
	fileParameter0["imageRatio3"] >> xmlFileState.cameraState[3].ratio;
	fileParameter0.release();

	FileStorage fileParameter1(xmlPath1, FileStorage::READ);

	if (fileParameter1.isOpened() == 0)
	{
		cout << "Parameter1 open failed" << endl;
		return -1;
	}
	fileParameter1["Matrix_H0"] >> xmlFileState.cameraState[0].homographyMatrix;
	fileParameter1["Matrix_H1"] >> xmlFileState.cameraState[1].homographyMatrix;
	fileParameter1["Matrix_H2"] >> xmlFileState.cameraState[2].homographyMatrix;
	fileParameter1["Matrix_H3"] >> xmlFileState.cameraState[3].homographyMatrix;

	//fileParameter1["AffMatrix_0"] >> xmlFileState.cameraState[0].affMatrix;
	//fileParameter1["AffMatrix_1"] >> xmlFileState.cameraState[1].affMatrix;
	//fileParameter1["AffMatrix_2"] >> xmlFileState.cameraState[2].affMatrix;
	//fileParameter1["AffMatrix_3"] >> xmlFileState.cameraState[3].affMatrix;

	fileParameter1.release();

	FileStorage fileParameter2(xmlPath2, FileStorage::READ);
	if (fileParameter2.isOpened() == 0)
	{
		cout << "Parameter2 open failed" << endl;
		return -1;
	}

	fileParameter2["F_i_tran"] >> xmlFileState.fusingState.translationCoordinate[0];
	fileParameter2["F_j_tran"] >> xmlFileState.fusingState.translationCoordinate[1];
	fileParameter2["B_i_tran"] >> xmlFileState.fusingState.translationCoordinate[6];
	fileParameter2["B_j_tran"] >> xmlFileState.fusingState.translationCoordinate[7];
	fileParameter2["R_i_tran"] >> xmlFileState.fusingState.translationCoordinate[4];
	fileParameter2["R_j_tran"] >> xmlFileState.fusingState.translationCoordinate[5];
	fileParameter2["L_j_tran"] >> xmlFileState.fusingState.translationCoordinate[3];

	fileParameter2["RIO_Hpoint0"] >> xmlFileState.cameraState[0].homographyPoint;
	fileParameter2["RIO_Hpoint1"] >> xmlFileState.cameraState[1].homographyPoint;
	fileParameter2["RIO_Hpoint2"] >> xmlFileState.cameraState[2].homographyPoint;
	fileParameter2["RIO_Hpoint3"] >> xmlFileState.cameraState[3].homographyPoint;

	fileParameter2["RIO_pinjie"] >> xmlFileState.fusingState.jiontPoint;

	fileParameter2.release();

	FileStorage fileParameter3(xmlPath3, FileStorage::READ);
	if (fileParameter3.isOpened() == 0)
	{
		cout << "Parameter3 open failed" << endl;
		return -1;
	}
	vector<string> xml2Tags = { "F_subavgB", "F_subavgG","F_subavgR","L_subavgB","L_subavgG","L_subavgR","R_subavgB","R_subavgG","R_subavgR","B_subavgB","B_subavgG","B_subavgR" };
	for (int i = 0; i<4; i++)
		for (int j = 0; j < 3; j++)
		{
			fileParameter2[xml2Tags[i * 3 + j]] >> xmlFileState.cameraState[i].colorDiff[j];
		}

	fileParameter3.release();

	return 0;
}