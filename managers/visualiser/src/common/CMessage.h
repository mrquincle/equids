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

typedef int vocab_t;

#define VOCAB(a,b,c,d) ((((int)(d))<<24)+(((int)(c))<<16)+(((int)(b))<<8)+((int)(a)))
#define VOCAB4(a,b,c,d) VOCAB((a),(b),(c),(d))
#define VOCAB3(a,b,c) VOCAB((a),(b),(c),(0))
#define VOCAB2(a,b) VOCAB((a),(b),(0),(0))
#define VOCAB1(a) VOCAB((a),(0),(0),(0))

#define V_ACTIONSELECTION VOCAB('a','c','t','s');
#define V_AVOIDIR VOCAB4('a','v','i','r');
#define V_BACKANDFORTH VOCAB3('b','a','f');
#define V_CAMERADETECTION VOCAB3('c','a','m');
#define V_DOCKING VOCAB4('d','o','c','k');
#define V_MAPPING VOCAB3('m','a','p');
#define V_LASERSCAN VOCAB4('l','a','s','r');
#define V_MOTORCALIBRATION VOCAB4('m','o','t','c');
#define V_REMOTECONTROL VOCAB2('r','c');
#define V_UBIPOSITION VOCAB3('u','b','i');
#define V_WENGUO VOCAB3('w','n','g');
#define V_ZIGBEEMSG VOCAB4('z','i','g','b');
#define V_MOVE_TO_POSITION VOCAB4('m','v','t','p');
#define V_ORGANISM_CONTROL VOCAB4('o','r','g','c');

#define MESSAGE_LENGTH  

typedef enum
{
	MSG_NONE = 0, // line 4 in CMessage.cpp
	MSG_START,
	MSG_STOP, // line 6
	MSG_RESET,
	MSG_QUIT,
	MSG_ACKNOWLEDGE,
	MSG_INIT,
	MSG_SPEED,
	MSG_HINGE,
	MSG_POS,
	DAEMON_MSG_RECRUITING,
	DAEMON_MSG_SEEDING,
	DAEMON_MSG_DOCKING, // line 16
	DAEMON_MSG_NEIGHBOUR_IP_REQ,
	DAEMON_MSG_NEIGHBOUR_IP,
	DAEMON_MSG_SEED_IP_REQ,
	DAEMON_MSG_SEED_IP,
	DAEMON_MSG_ALLROBOTS_IP_REQ,
	DAEMON_MSG_ALLROBOTS_IP,
	DAEMON_MSG_PROGRESS_REQ,
	DAEMON_MSG_PROGRESS,
	DAEMON_MSG_DISASSEMBLY,
	DAEMON_MSG_STATE_REQ, // line 26
	DAEMON_MSG_STATE,
	MSG_CAM_VIDEOSTREAM_STOP,
	MSG_CAM_VIDEOSTREAM_START,
	MSG_CAM_DETECT_DOCKING,
	MSG_CAM_DETECT_MAPPING,
	MSG_CAM_DETECT_STAIR,
	MSG_CAM_DETECTED_BLOB,
	MSG_CAM_DETECTED_BLOB_ARRAY,
	MSG_CAM_DETECTED_STAIR,
	MSG_LASER_DETECT_STEP, // line 36
	MSG_MOTOR_CALIBRATION_RESULT,
	MSG_GET_POSITION,
	MSG_SET_POSITION,
	MSG_UBISENCE_POSITION,
	MSG_MAP_DATA,
	MSG_GET_ALL_MAPPED_OBJS,
	MSG_GET_NEAREST_MAPPED_OBJECT_OF_TYPE_TO_POS,
	MSG_MAP_COVARIANCE,
	MSG_MAP_COMPLETE,
	MSG_CALIBRATE, // line 46
	MSG_ZIGBEE_MSG,
	MSG_MOVETOPOSITION,
	MSG_MOVETOPOSITION_DONE,
	MSG_SOCKET_DOCKING_DONE,
	MSG_FORCE_CHANGE_JOCKEY,
	MSG_REMOTE_CONTROL,
	MSG_INIT_ORGANISM,
	MSG_LEADER,
	MSG_DOCK_ORGANISM,
	MSG_COLLISION_DETECTED, // line 56
	MSG_FIND_DONE,
	MSG_FIND_FAIL,
	MSG_ACTIVE_JOCKEYS,
	MSG_CAM_TURN_ON_ALL_THE_TIME,
	MSG_NEED_ORG,
	MSG_HELP_ORG,
	MSG_HELP_ACP,
	MSG_MY_ZIGBEE_ID,
	TOTAL_NUMBER_OF_MESSAGES // for debugging
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
	int fromRobot;
	TMessageType type;
	int len;
	bool valid;
	uint8_t *data;
};

#endif

/* end of CMessage.h */
