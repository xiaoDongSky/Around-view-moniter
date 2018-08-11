#include "configuration.h"
#define AVM_TEST

void configurationReading(ConfigParameters &parameters)
{
	boost::property_tree::ptree iniConfig,modePara,imageSizePara,xmlPathPara, BGRFTPPara, BGRRTPPara, BGRTTPPara, 
		YUYVFTPPara, YUYVRTPPara, YUYVTTPPara, UYVYFTPPara, UYVYRTPPara, UYVYTTPPara, numberPara;
	string iniFilePath = "..\\Configuration.ini";
	boost::property_tree::ini_parser::read_ini(iniFilePath, iniConfig);

	modePara = iniConfig.get_child("mode");
	imageSizePara = iniConfig.get_child("imageSize");
	xmlPathPara = iniConfig.get_child("xmlPath");
	numberPara = iniConfig.get_child("numberPara");

	BGRFTPPara = iniConfig.get_child("bgrFundamental");
	BGRRTPPara = iniConfig.get_child("bgrRectangle");
	BGRTTPPara = iniConfig.get_child("bgrTriangle");

	YUYVFTPPara = iniConfig.get_child("yuyvFundamental");
	YUYVRTPPara = iniConfig.get_child("yuyvTectangle");
	YUYVTTPPara = iniConfig.get_child("yuyvTriangle");

	UYVYFTPPara = iniConfig.get_child("uyvyFundamental");
	UYVYRTPPara = iniConfig.get_child("uyvyTectangle");
	UYVYTTPPara = iniConfig.get_child("uyvyTriangle");

	parameters.testFlag = modePara.get<int>("test", 0);
	parameters.colorSpace = modePara.get<char>("colorSpace", "bgr");

	parameters.sourceImageWidth = imageSizePara.get<int>("sourceImageWidth", 720);
	parameters.sourceImageHeight = imageSizePara.get<int>("sourceImageHeight", 480);
	parameters.targetImageWidth = imageSizePara.get<int>("targetImageWidth", 720);
	parameters.targetImageHeight = imageSizePara.get<int>("targetImageHeight", 720);

	string commonPath, fileName;
	commonPath = xmlPathPara.get<char>("path", "..\\..\\Parameter");
	fileName = xmlPathPara.get<char>("fileName1", "parameter0.xml");
	parameters.xmlPath[0] = commonPath + "\\" + fileName;
	fileName = xmlPathPara.get<char>("fileName2", "parameter1.xml");
	parameters.xmlPath[1] = commonPath + "\\" + fileName;
	fileName = xmlPathPara.get<char>("fileName3", "parameter2.xml");
	parameters.xmlPath[2] = commonPath + "\\" + fileName;
	fileName = xmlPathPara.get<char>("fileName4", "parameter3.xml");
	parameters.xmlPath[3] = commonPath + "\\" + fileName;

	parameters.BGRNumberPath = numberPara.get<char>("bgrNumber","..\\..\\BGRTable\\Parameters\\AVMNumbers" );
	parameters.YUYVNumberPath = numberPara.get<char>("yuyvNumber", "..\\..\\YUYVTable\\Parameters\\AVMNumbers");
	parameters.UYVYNumberPath = numberPara.get<char>("uyvyNumber", "..\\..\\UYVYTable\\Parameters\\AVMNumbers");

	commonPath = BGRFTPPara.get<char>("fundamentalPath", "..\\..\\360ViewExcel\\FundamentalExcel");
	fileName = BGRFTPPara.get<char>("fileName1", "FundamentalSrcRow");
	parameters.BGRFundamentalPath[0] = commonPath + "\\" + fileName;
	fileName = BGRFTPPara.get<char>("fileName2", "FundamentalSrcCol");
	parameters.BGRFundamentalPath[1] = commonPath + "\\" + fileName;
	fileName = BGRFTPPara.get<char>("fileName3", "FundamentalDstRow");
	parameters.BGRFundamentalPath[2] = commonPath + "\\" + fileName;
	fileName = BGRFTPPara.get<char>("fileName4", "FundamentalDstCol");
	parameters.BGRFundamentalPath[3] = commonPath + "\\" + fileName;

	commonPath = BGRRTPPara.get<char>("rectanglePath", "..\\..\\360ViewExcel\\FusedRectangleExcel");
	fileName = BGRRTPPara.get<char>("fileName1", "RectangleDstRow");
	parameters.BGRRectanglePath[0] = commonPath + "\\" + fileName;
	fileName = BGRRTPPara.get<char>("fileName2", "RectangleDstCol");
	parameters.BGRRectanglePath[1] = commonPath + "\\" + fileName;
	fileName = BGRRTPPara.get<char>("fileName3", "RectangleSrc1Row");
	parameters.BGRRectanglePath[2] = commonPath + "\\" + fileName;
	fileName = BGRRTPPara.get<char>("fileName4", "RectangleSrc1Col");
	parameters.BGRRectanglePath[3] = commonPath + "\\" + fileName;
	fileName = BGRRTPPara.get<char>("fileName5", "RectangleSrc2Row");
	parameters.BGRRectanglePath[4] = commonPath + "\\" + fileName;
	fileName = BGRRTPPara.get<char>("fileName6", "RectangleSrc2Col");
	parameters.BGRRectanglePath[5] = commonPath + "\\" + fileName;
	fileName = BGRRTPPara.get<char>("fileName7", "RectangleDistance");
	parameters.BGRRectanglePath[6] = commonPath + "\\" + fileName;

	commonPath = BGRTTPPara.get<char>("trianglePath", "..\\..\\360ViewExcel\\FusedTriangleExcel");
	fileName = BGRTTPPara.get<char>("fileName1", "TriangleDstRow");
	parameters.BGRTrianglePath[0] = commonPath + "\\" + fileName;
	fileName = BGRTTPPara.get<char>("fileName2", "TriangleDstCol");
	parameters.BGRTrianglePath[1] = commonPath + "\\" + fileName;
	fileName = BGRTTPPara.get<char>("fileName3", "TriangleSrc1Row");
	parameters.BGRTrianglePath[2] = commonPath + "\\" + fileName;
	fileName = BGRTTPPara.get<char>("fileName4", "TriangleSrc1Col");
	parameters.BGRTrianglePath[3] = commonPath + "\\" + fileName;
	fileName = BGRTTPPara.get<char>("fileName5", "TriangleSrc2Row");
	parameters.BGRTrianglePath[4] = commonPath + "\\" + fileName;
	fileName = BGRTTPPara.get<char>("fileName6", "TriangleSrc2Col");
	parameters.BGRTrianglePath[5] = commonPath + "\\" + fileName;
	fileName = BGRTTPPara.get<char>("fileName7", "TriangleDistance");
	parameters.BGRTrianglePath[6] = commonPath + "\\" + fileName;

	commonPath = YUYVFTPPara.get<char>("fundamentalPath", "..\\..\\YUYVTable\\FundamentalExcel");
	fileName = YUYVFTPPara.get<char>("fileName1", "FundamentalDst");
	parameters.YUYVFundamentalPath[0] = commonPath + "\\" + fileName;
	fileName = YUYVFTPPara.get<char>("fileName2", "FundamentalSrc");
	parameters.YUYVFundamentalPath[1] = commonPath + "\\" + fileName;
	
	commonPath = YUYVRTPPara.get<char>("rectanglePath", "..\\..\\YUYVTable\\FusedRectangleExcel");
	fileName = YUYVRTPPara.get<char>("fileName1", "RectangleDst");
	parameters.YUYVRectanglePath[0] = commonPath + "\\" + fileName;
	fileName = YUYVRTPPara.get<char>("fileName2", "RectangleSrc1");
	parameters.YUYVRectanglePath[1] = commonPath + "\\" + fileName;
	fileName = YUYVRTPPara.get<char>("fileName3", "RectangleSrc2");
	parameters.YUYVRectanglePath[2] = commonPath + "\\" + fileName;
	fileName = YUYVRTPPara.get<char>("fileName4", "RectangleDistance");
	parameters.YUYVRectanglePath[3] = commonPath + "\\" + fileName;

	commonPath = YUYVTTPPara.get<char>("trianglePath", "..\\..\\YUYVTable\\FusedTriangleExcel");
	fileName = YUYVTTPPara.get<char>("fileName1", "Triangle1Dst");
	parameters.YUYVTrianglePath[0] = commonPath + "\\" + fileName;
	fileName = YUYVTTPPara.get<char>("fileName2", "Triangle1Src1");
	parameters.YUYVTrianglePath[1] = commonPath + "\\" + fileName;
	fileName = YUYVTTPPara.get<char>("fileName3", "Triangle1Src2");
	parameters.YUYVTrianglePath[2] = commonPath + "\\" + fileName;
	fileName = YUYVTTPPara.get<char>("fileName4", "TriangleDistance");
	parameters.YUYVTrianglePath[3] = commonPath + "\\" + fileName;


	commonPath = UYVYFTPPara.get<char>("fundamentalPath", "..\\..\\UYVYTable\\FundamentalExcel");
	fileName = UYVYFTPPara.get<char>("fileName1", "FundamentalDst");
	parameters.UYVYFundamentalPath[0] = commonPath + "\\" + fileName;
	fileName = UYVYFTPPara.get<char>("fileName2", "FundamentalSrc");
	parameters.UYVYFundamentalPath[1] = commonPath + "\\" + fileName;

	commonPath = UYVYRTPPara.get<char>("rectanglePath", "..\\..\\UYVYTable\\FusedRectangleExcel");
	fileName = UYVYRTPPara.get<char>("fileName1", "RectangleDst");
	parameters.UYVYRectanglePath[0] = commonPath + "\\" + fileName;
	fileName = UYVYRTPPara.get<char>("fileName2", "RectangleSrc1");
	parameters.UYVYRectanglePath[1] = commonPath + "\\" + fileName;
	fileName = UYVYRTPPara.get<char>("fileName3", "RectangleSrc2");
	parameters.UYVYRectanglePath[2] = commonPath + "\\" + fileName;
	fileName = UYVYRTPPara.get<char>("fileName4", "RectangleDistance");
	parameters.UYVYRectanglePath[3] = commonPath + "\\" + fileName;

	commonPath = UYVYTTPPara.get<char>("trianglePath", "..\\..\\UYVYTable\\FusedTriangleExcel");
	fileName = UYVYTTPPara.get<char>("fileName1", "Triangle1Dst");
	parameters.UYVYTrianglePath[0] = commonPath + "\\" + fileName;
	fileName = UYVYTTPPara.get<char>("fileName2", "Triangle1Src1");
	parameters.UYVYTrianglePath[1] = commonPath + "\\" + fileName;
	fileName = UYVYTTPPara.get<char>("fileName3", "Triangle1Src2");
	parameters.UYVYTrianglePath[2] = commonPath + "\\" + fileName;
	fileName = UYVYTTPPara.get<char>("fileName4", "TriangleDistance");
	parameters.UYVYTrianglePath[3] = commonPath + "\\" + fileName;
}