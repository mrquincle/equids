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
   "Pos",
   "Recruiting",
   "Seeding",
   "Docking",
   "Neighbour's IP REQ",
   "Neighbour's IP",
   "Seed's IP REQ",
   "Seed's IP",
   "AllRobot's IP REQ",
   "AllRobot's IP",
   "Progress REQ",
   "Progress",
   "Disassembly",
   "State REQ",
   "State",
   "Stop video Stream",
   "Start video Stream",
   "Start detect docking",
   "Start detect mapping",
   "Start detect stair",
   	"Detected blob",
   	"Detected blob array",
   	"Detected stair",
   	"Results of motor calibration",
   	"Robot position from ubisence",
      "Map data",
      "Map covariance",
      "Not used"
};


CMessage::CMessage()
{

	type = MSG_NONE;
	value1= value2 = value3 = 0;
   data = NULL;
}

CMessage::~CMessage()
{
   /*if (data!=NULL) {
      delete[] data;
      data = NULL;
   }
   */
}

const char * CMessage::getStrType()
{
	return StrMessage[type];
}

void CMessage::set(const ELolMessage*msg) {
   type = (TMessageType)msg->command;
   len = msg->length;
   if (len>0) {
      data = new uint8_t[len];
      memcpy(data,msg->data,msg->length);
   } else {
      data = NULL;
   }
}

void CMessage::set(const CMessage *msg) {
   type = (TMessageType)msg->type;
   len = msg->len;
   if (len>0) {
      data = new uint8_t[len];
      memcpy(data,msg->data,msg->len);
   } else {
      data = NULL;
   }
}
