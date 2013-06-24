#include "CMessage.h"

const char* StrMessage[] ={
	"None",
	"Start",
	"Stop",
	"Reset",
	"Quit",
	"Speed set",
	"Move hinge",
	"Number"
};

CMessage::CMessage()
{
	type = MSG_NONE;
	value1= value2 = 0;
}

CMessage::~CMessage()
{
}

const char * CMessage::getStrType()
{
	return StrMessage[type];
}

void CMessage::pack()
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
	if (value3 < 0) {
		value3 = - value3;
		buf[9] = 1;
	}else{
		buf[9] = 0;
	}
	buf[7] = value3%256; 
	buf[8] = value3/256;
}

void CMessage::unpack()
{
	type = (TMessageType) buf[0];
	value1 = buf[1] + buf[2]*256; 
	value2 = buf[4] + buf[5]*256; 
	value3 = buf[7] + buf[8]*256; 
	if (buf[3] > 0) value1 = - value1;
	if (buf[6] > 0) value2 = - value2;
	if (buf[9] > 0) value2 = - value2;
}

