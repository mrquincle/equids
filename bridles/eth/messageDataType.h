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

struct MotorCalibResult {//same meaning as for motors
	double odometry_koef1;
	double odometry_koef2;
	double odometry_koef3;
	int calibratedSpeed;
};

#endif /* MESSAGEDATATYPE_H_ */

