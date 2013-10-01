#ifndef __MESSAGEDATATYPE_H__
#define __MESSAGEDATATYPE_H__
#include <stdint.h>
#define MAX_DOCKING_PATTERNS 2

struct DetectedBlob {
	float x;
	float y;
	float z;
	float phi;
};


typedef enum {
	S_START,
	S_MAPPING,
	S_REMOTE_CONTROL,
	S_EXPLORATION,
	S_DETECT_OBJECT,
	S_OBJECT_DETECTED,
	S_RECRUITING,
    S_CONNECTING,
	S_ASSEMBLE,
	S_MACROLOCOMOTION,
	S_CLIMB_STEP,
	S_DISASSEMBLE_5_TO_3,
	S_QUIT
} TStateMST;


struct DetectedBlobWSize {
	uint8_t size;
	DetectedBlob detectedBlob;
};

struct DetectedBlobWSizeArray {
	uint8_t size;
	DetectedBlob detectedBlobArray[MAX_DOCKING_PATTERNS];
};

struct MotorCalibResult { //same meaning as for motors
	float odometry_koef1;
	float odometry_koef2;
	float odometry_koef3;
	int calibratedSpeed;
};

struct UbiPosition {
	float x; //!< The x coordinate
	float y; //!< The y coordinate
	float z; //!< The y coordinate
	uint32_t time_stamp; //!< The Ubisense coordinate time
};

enum MapObjectType {
	NORMAL_CIRCLE = 0,
	DOCK_CIRCLE,
	DOCK_CIRCLE_ORGANISM,
	SMALL_STEP,
	LARGE_STEP,
	WALL,
	ROBOT,
	UNIDENTIFIED
};

static const char* StrMapObjectType[] = {
		"Normal circle",
		"Dock circle",
		"Dock circle on organism",
		"Small step",
		"Large step",
		"Wall",
		"Robot",
		"Unidentified"
};

/**
 * Anne: I assume phi is rotation in x,y plane, running from [-pi,+pi] with [x,y]=[1,0] corresponding with phi=0
 */
struct MappedObjectPosition {
	int mappedBy;
	MapObjectType type;
	int map_id;
	float xPosition; //!< The x coordinate
	float yPosition; //!< The y coordinate
	float zPosition; //!< The z coordinate
	float phiPosition; //!< The phi coordinate
	float xUncertainty; //!< The x coordinate Uncertainty
	float yUncertainty; //!< The y coordinate Uncertainty
	float zUncertainty; //!< The z coordinate Uncertainty
	float phiUncertainty; //!< The phi coordinate Uncertainty
};

struct NearestObjectOfTypeToThisPosition{
	float xPosition; //!< The x coordinate
	float yPosition; //!< The y coordinate
	float phiPosition; //!< The phi coordinate
	MapObjectType type;
};

struct RobotPosition{
	float x; //!< The x coordinate
	float y; //!< The y coordinate
	float phi; //!< The phi coordinate
};

struct MotorCommand {
	int16_t forward;
	int16_t radius;
} __attribute__((packed));

typedef enum {
	A_NO_ACTION = 0,
	A_CLOSE_DOCK_FRONT,
	A_CLOSE_DOCK_LEFT,
	A_CLOSE_DOCK_RIGHT,
	A_CLOSE_DOCK_REAR,
	A_MOVE_HINGE,
	A_MOVE_DOCKING,
	A_SET_LEDS_COLOR
} RemoteRobotAction;

struct RemoteControlData {
	int8_t speed1;
	int8_t speed2;
	int8_t speed3;
	bool directMotorSpeed; // if to use directly motor speed or use motor->setSpeed() == false
	int8_t hingeAngle; //if action is A_MOVE_HINGE thne moce hinge to hingeAngle
	int8_t dockingAngleLeft; //if action is A_MOVE_DOCKING  can be set in -90 - 90 ,
	int8_t dockingAngleRight;
	uint8_t source;
	uint8_t ledcolor;
	RemoteRobotAction action;
};

union IP_rob {
    unsigned int ip;
    struct {
      unsigned char d;
      unsigned char c;
      unsigned char b;
      unsigned char a;
    } ip2;
};
#endif /* MESSAGEDATATYPE_H_ */

