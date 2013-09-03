/*
 * Mapping.h
 *
 *  Created on: 22.7.2013
 *      Author: robert
 */

#ifndef MAPPING_H_
#define MAPPING_H_
#include "../motor/CMotors.h"
#include "../common/cmath.h"

class Mapping {
public:
	enum MappingState {
		START_TURNING_AROUND,
		END_TURNING_AROUND,
		DRIVE_FORWARD,
		TURNING_TO_DIRECTION,
		START_MAPPING
	};
	Mapping(CMotors* motors);
	virtual ~Mapping();
	void doMappingMotion(bool seeBlob);
	int wait_stopped; // wait stopped until wait_stopped==0
	int seedSameBlob;
	int image_wait_count;
	int runs;
	MappingState actualState; //switching between robot motion behavior
private:
	CMotors* motor;
	double lastPosition[5]; //for storing position of robot in previous important position

	int stai_in_motion; // continue turning until ==0
	double turnToDirectionAngle;

	CTimer* timer;
};

#endif /* MAPPING_H_ */
