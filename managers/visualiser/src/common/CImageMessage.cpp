#include "CImageMessage.h"

const char* StrImageMessage[] ={
	"None",
	"Get image",
	"Learn",
	"Reset",
	"Save",
	"Pause",
	"Resume",
	"Step",
	"Number"
};

CImageMessage::CImageMessage()
{
	type = MSGI_NONE;
	value1= value2 = 0;
}

CImageMessage::~CImageMessage()
{
}

const char * CImageMessage::getStrType()
{
	return StrImageMessage[type];
}

void CImageMessage::pack()
{
	buf[0] = type;
	if (value1 < 0) {
		value1 = - value1;
		buf[3] = 1;
	}else{
		buf[3] = 0;
	}
	buf[1] = value1%256; 
	buf[2] = value1/256;
	if (value2 < 0) {
		value2 = - value2;
		buf[6] = 1;
	}else{
		buf[6] = 0;
	}
	buf[4] = value2%256; 
	buf[5] = value2/256;
}

void CImageMessage::unpack()
{
	type = (TImageMessageType)buf[0];
	value1 = buf[1] + buf[2]*256; 
	value2 = buf[4] + buf[5]*256; 
	if (buf[3] > 0) value1 = - value1;
	if (buf[6] > 0) value2 = - value2;
}

