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
	enum MappingState
	  {
	      START_TURNING_AROUND,
	      END_TURNING_AROUND,
	      DRIVE_FORWARD,
	      TURNING_TO_DIRECTION,
	      START_MAPPING
	  };
	Mapping(CMotors* motors);
	virtual ~Mapping();
	void doMappingMotion();
	int wait_stopped; // wait stopped until wait_stopped==0
private:
	CMotors* motor;
	double lastPosition[5];//for storing position of robot in previous important position
	MappingState actualState;//switching between robot motion behavior

	int stai_in_motion; // continue turning until ==0
	double turnToDirectionAngle;
};

#endif /* MAPPING_H_ */
