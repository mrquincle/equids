/*
 * File name: CMessage.h
 * Date:      2006/10/12 11:56
 * Author:    
 */

#ifndef __CMESSAGE_H__
#define __CMESSAGE_H__

#include "ethlolmsg.h"

#define MESSAGE_LENGTH  

typedef enum
{
   MSG_NONE = 0,
   MSG_START,
   MSG_STOP,
   MSG_RESET,
   MSG_QUIT,
   MSG_ACKNOWLEDGE,
   MSG_INIT,
   MSG_SPEED,
   MSG_HINGE,
   MSG_POS,
   DAEMON_MSG_RECRUITING,
   DAEMON_MSG_SEEDING,
   DAEMON_MSG_DOCKING,
   DAEMON_MSG_NEIGHBOUR_IP_REQ,
   DAEMON_MSG_NEIGHBOUR_IP,
   DAEMON_MSG_SEED_IP_REQ,
   DAEMON_MSG_SEED_IP,
   DAEMON_MSG_ALLROBOTS_IP_REQ,
   DAEMON_MSG_ALLROBOTS_IP,
   DAEMON_MSG_PROGRESS_REQ,
   DAEMON_MSG_PROGRESS,
   DAEMON_MSG_DISASSEMBLY,
   DAEMON_MSG_STATE_REQ,
   DAEMON_MSG_STATE,
   MSG_CAM_VIDEOSTREAM_STOP,
   MSG_CAM_VIDEOSTREAM_START,
   MSG_CAM_DETECT_DOCKING,
   MSG_CAM_DETECT_MAPPING,
   MSG_CAM_DETECT_STAIR,
   MSG_CAM_DETECTED_BLOB,
   MSG_CAM_DETECTED_BLOB_ARRAY,
   MSG_CAM_DETECTED_STAIR,
   MSG_MOTOR_CALIBRATION_RESULT,
   MSG_UBISENCE_POSITION,
   MSG_MAP_DATA,
   MSG_MAP_COVARIANCE,
   MSG_NUMBER,
} TMessageType;

extern const char* StrMessage[];

class CMessage
{
	public:
		CMessage();
		~CMessage();
		unsigned char buf[];
		void pack();
		void unpack();
        	void set(const ELolMessage*msg);
		const char* getStrType();
		TMessageType type;
		int len;
		const uint8_t *data;
		int value1;
		int value2;
		int value3;
};

#endif

/* end of CMessage.h */
