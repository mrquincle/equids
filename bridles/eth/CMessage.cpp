#include "CMessage.h"

const char* StrMessage[] ={
   "None",
   "Start",
   "Stop",
   "Reset",
   "Quit",
   "Acknowledgment",
   "Init",
   "Speed set",
   "Move hinge",
   "Number"
};


CMessage::CMessage()
{
	type = MSG_NONE;
	value1= value2 = value3 = 0;
}

CMessage::~CMessage()
{
}

const char * CMessage::getStrType()
{
	return StrMessage[type];
}

void CMessage::set(const ELolMessage*msg) {
   type = (TMessageType)msg->command;
   data = msg->data;
   len = msg->length;
   unpack();
}

void CMessage::pack()
{
   buf[0] = value1%256;
   buf[1] = value1/256;
   if (value1 < 0) {
      value1 = - value1;
      buf[2] = 1;
   }else{
      buf[2] = 0;
   }

   buf[3] = value2%256;
   buf[4] = value2/256;
   if (value2 < 0) {
      value2 = - value2;
      buf[5] = 1;
   }else{
      buf[5] = 0;
   }
   buf[6] = value3%256;
   buf[7] = value3/256;
   if (value3 < 0) {
      value3 = - value3;
      buf[8] = 1;
   } else {
      buf[8] = 0;
   }
}

void CMessage::unpack()
{
   if (len>3) {
      value1 = buf[0] + buf[1]*256;
      if (buf[2] > 0) {
         value1 = - value1;
      }
   }
   if (len>6) {
      value2 = buf[3] + buf[4]*256;
      if (buf[5] > 0) {
         value2 = - value2;
      }
   }
   if (len>9) {
      value3 = buf[6] + buf[7]*256;
      if (buf[8] > 0) {
         value2 = - value2;

      }
   }
}

