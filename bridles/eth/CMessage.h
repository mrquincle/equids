/*
 * File name: CMessage.h
 * Date:      2006/10/12 11:56
 * Author:    
 */

#ifndef __CMESSAGE_H__
#define __CMESSAGE_H__

#include "ethlolmsg.h"
#include <string.h>
#include <stdio.h>

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
   MSG_LASER_DETECT_STEP, // use the laser to detect the step
   MSG_MOTOR_CALIBRATION_RESULT,
   MSG_GET_POSITION,
   MSG_SET_POSITION,
   MSG_UBISENCE_POSITION,
   MSG_MAP_DATA,
   MSG_GET_ALL_MAPPED_OBJS,
   MSG_GET_NEAREST_MAPPED_OBJECT_OF_TYPE_TO_POS,
   MSG_MAP_COVARIANCE,
   MSG_MAP_COMPLETE,
   MSG_CALIBRATE,
   MSG_NUMBER,
   MSG_ZIGBEE_MSG,
   MSG_MOVETOPOSITION,
   MSG_MOVETOPOSITION_DONE,
   MSG_SOCKET_DOCKING_DONE,
   MSG_FORCE_CHANGE_JOCKEY,
   MSG_REMOTE_CONTROL,
   MSG_INIT_ORGANISM,
   MSG_LEADER,
   MSG_DOCK_ORGANISM
} TMessageType;

extern const char* StrMessage[];

class CMessage
{
	public:
		CMessage();
		~CMessage();
        void set(const ELolMessage*msg);
        void set(const CMessage *msg);
		const char* getStrType();
		static CMessage unpackZBMessage(CMessage ZBmessage);
		static CMessage packToZBMessage(uint64_t ubitag, int type, void *data,	int len);
		TMessageType type;
		int len;
		bool valid;
		uint8_t *data;
};

#endif

/* end of CMessage.h */
