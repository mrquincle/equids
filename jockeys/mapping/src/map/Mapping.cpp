/*
 * Mapping.cpp
 *
 *  Created on: 22.7.2013
 *      Author: robert
 */

#include "Mapping.h"
#define RANDOM_MOTION
#define IMAGE_WAIT_COUNT 50
#define TURNING_COUNT 2
#define MINIMUM_SEE_BLOB_REQ 20
#define DRIVE_FORWARD_COUNT 5
#define DRIVE_FORWARD_DIST 0.4

Mapping::Mapping(CMotors* motors) {
	// TODO Auto-generated constructor stub
	this->motor = motors;
	this->actualState = START_MAPPING;
	this->wait_stopped = IMAGE_WAIT_COUNT;
	this->stai_in_motion = TURNING_COUNT;
	this->seedSameBlob=0;
	image_wait_count=IMAGE_WAIT_COUNT;
	//this->timer=new CTimer();
	//this->timer->start();
}

Mapping::~Mapping() {

}

/**
 * Do mapping pseudo random map procedure in constant sequence - drive forward -> turn around start -> turn around end -> turn rand angle -> drive forward .....
 */
void Mapping::doMappingMotion(bool seeBlob) {

	if(seeBlob){
		seedSameBlob+=1;
	}

	switch (this->actualState) {
	case START_TURNING_AROUND: {
	//	printf("START_TURNING_AROUND\n");
		if (this->wait_stopped > 0) {
		//	printf("wait stopped\n");
			this->wait_stopped -= 1;
			motor->setSpeeds(0, 0);
		} else {
		//	printf("moving\n");
			if (normalizeAngleDiff(
					this->lastPosition[2] - motor->getPosition()[2])
					< M_PI / 2) {
				//continue without changes
			} else {
				//switch to END_TURNING_ARROUNG where is switched to turn rand agle if angle differenc is around null
				this->actualState = END_TURNING_AROUND;
			}
			if (this->stai_in_motion > 0) {
				this->stai_in_motion -= 1;
			} else {
				this->wait_stopped = IMAGE_WAIT_COUNT;
				this->stai_in_motion = TURNING_COUNT;
			}
			this->seedSameBlob=0;
			motor->setSpeeds(0, motor->calibratedSpeed);

		}
	//	printf("stai_turning %d \n", this->stai_in_motion);
	//	printf("wait stopped %d \n", this->wait_stopped);
	}
		break;
	case END_TURNING_AROUND: {
	//	printf("END_TURNING_AROUND\n");
		if (this->wait_stopped > 0) {
			this->wait_stopped -= 1;
			motor->setSpeeds(0, 0);
		} else {
			if (normalizeAngleDiff(
					this->lastPosition[2] - motor->getPosition()[2])
					< M_PI / 4) {
				this->actualState = TURNING_TO_DIRECTION;
				memcpy(lastPosition, motor->getPosition(), 5 * sizeof(double));
				//set turnToDirectionAngle
#if defined(RANDOM_MOTION)
				this->turnToDirectionAngle = randFromTO(-M_PI, M_PI);
#else

#endif
			}
			if (this->stai_in_motion > 0) {
				this->stai_in_motion -= 1;
			} else {
				this->wait_stopped = IMAGE_WAIT_COUNT;
				this->stai_in_motion = TURNING_COUNT;
			}
			this->seedSameBlob=0;
			motor->setSpeeds(0, motor->calibratedSpeed);

		}
	}
		break;
	case DRIVE_FORWARD: {
	//	printf("DRIVE_FORWAR\n");
		if (this->wait_stopped > 0) {
			this->wait_stopped -= 1;
			motor->setSpeeds(0, 0);
		} else {
			if (this->stai_in_motion > 0) {
				this->stai_in_motion -= 1;
			} else {
				this->wait_stopped = IMAGE_WAIT_COUNT;
				this->stai_in_motion = DRIVE_FORWARD_COUNT;
			}
			if (euclideanDistance(this->lastPosition,
					motor->getPosition()) > DRIVE_FORWARD_DIST) {
				this->actualState = START_TURNING_AROUND;
				memcpy(lastPosition, motor->getPosition(), 5 * sizeof(double));
				motor->setSpeeds(0, 0);
			} else {
				this->seedSameBlob=0;
				motor->setSpeeds(motor->calibratedSpeed, 0);
			}
		}
	}
		break;
	case TURNING_TO_DIRECTION: {
	//	printf("TURNING_TO_DIRECTION\n");
		if (this->wait_stopped > 0) {
			this->wait_stopped -= 1;
			motor->setSpeeds(0, 0);
		} else {
			if (this->stai_in_motion > 0) {
				this->stai_in_motion -= 1;
			} else {
				this->wait_stopped = IMAGE_WAIT_COUNT;
				this->stai_in_motion = TURNING_TO_DIRECTION;
			}
			if (normalizeAngleDiff(
					this->turnToDirectionAngle - motor->getPosition()[2])
					< M_PI / 6) {
				memcpy(lastPosition, motor->getPosition(), 5 * sizeof(double));
				this->actualState = DRIVE_FORWARD;
				motor->setSpeeds(0, 0);
			} else {
				this->seedSameBlob=0;
				motor->setSpeeds(0, motor->calibratedSpeed);
			}
		}

	}
		break;
	case START_MAPPING: {
	//	printf("START_MAPPIN\n");
		memcpy(lastPosition, motor->getPosition(), 5 * sizeof(double));
		this->actualState = START_TURNING_AROUND;
	}
		break;
	default: {
		printf("no mapping state");
	}
		break;
	}


}

