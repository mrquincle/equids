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

#ifndef __TState__
#define __TState__
typedef enum {
	S_START = 0,
	S_CALIBRATE_ODOMETRY,
	S_BUILD_MAP,
	S_DOCK_SOCKET,
	S_DOCKED,
	S_STREAM_VIDEO,
	S_MAPPING_DETECTION,
	S_UBIPOS,
	S_REMOTE_CONTROL,
	S_DOCK_NOW,
	S_ORGANISM_REMOTECONTROL,
	S_ORGANISM_DOCKING,
	S_REMOTE_CONTROLLED_BY_LEADER,
	S_LEADER_OF_ORGANISM_DOCKING,
	S_LEADER_OF_ORGANISM_REMOTECONTROL,
	S_QUIT
} TState;
#endif

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
	NORMAL_CIRCLE, DOCK_CIRCLE, SMALL_STEP, LARGE_STEP, WALL, ROBOT, UNIDENTIFIED
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
};

struct MappedObjectCovariance {
	int mappedBy;
	MapObjectType type;
	int map_id;
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
	A_MOVE_DOCKING
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

