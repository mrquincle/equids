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

struct DetectedBlobWSize {
	uint8_t size;
	DetectedBlob detectedBlob;
};

struct DetectedBlobWSizeArray {
	uint8_t size;
	DetectedBlob detectedBlobArray[MAX_DOCKING_PATTERNS];
};

struct MotorCalibResult { //same meaning as for motors
	double odometry_koef1;
	double odometry_koef2;
	double odometry_koef3;
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
} __attribute__((packed));

struct MappedObjectCovariance {
	int mappedBy;
	MapObjectType type;
	int map_id;
	float xUncertainty; //!< The x coordinate Uncertainty
	float yUncertainty; //!< The y coordinate Uncertainty
	float zUncertainty; //!< The z coordinate Uncertainty
	float phiUncertainty; //!< The phi coordinate Uncertainty
};

/**
 * Send motor command as understood by CMotors.setRadianSpeeds
 */
struct MotorCommand {
	int16_t forward;
	int16_t radius;
} __attribute__((packed));

#endif /* MESSAGEDATATYPE_H_ */

