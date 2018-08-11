// AVMTableCreate.cpp : 定义控制台应用程序的入口点。
//
#include "AVMFusing.h"
#include "time.h"
#include "configuration.h"
int main()
{
	ConfigParameters iniPara;
	configurationReading(iniPara);
	XmlFileState xmlState;
	xmlFileRead(xmlState,iniPara.xmlPath[0], iniPara.xmlPath[1], iniPara.xmlPath[2], iniPara.xmlPath[3]);

	AVMFusing AVMState(xmlState, iniPara);

	if (iniPara.colorSpace == "bgr")
	{
		AVMState.creatBGRFiles();
		imwrite("aaa.bmp",AVMState.imageFused);
	}
	else if (iniPara.colorSpace == "yuyv")
	{
		AVMState.creatYUYVFiles();
	}
	else if (iniPara.colorSpace == "uyvy")
	{
		AVMState.creatUYVYFiles();
	}
	else
	{
		cout << "colorSpace error" << endl;
	}

    return 0;
}

