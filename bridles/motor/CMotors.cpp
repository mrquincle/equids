/*
 * CMotors.cpp
 *
 *  Created on: Dec 16, 2010
 *      Author: anne
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <CMotors.h>

CMotors::CMotors(RobotBase *robot_base, RobotBase::RobotType robot_type) {
	printf("Create motors object\n");
	if (robot_base == NULL) {
		fprintf(stderr, "robot_base is null, error in instantiation!\n");
	}
	this->robot_base = robot_base;
	this->robot_type = robot_type;
}

CMotors::~CMotors() {

}

/**
 * Initialize the motors. Enable high-power voltage that is required to drive the motors.
 */
void CMotors::init() {
	robot_base->EnableMotors(true);
}

void CMotors::setSpeeds(int forward, int turn) {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		fprintf(stderr, "To be implemented\n");
		ActiveWheel *bot = (ActiveWheel*)robot_base;
//		bot->moveForward(forward);
//		bot->moveRight(turn);
		break;
	}
	case RobotBase::KABOT: {
		fprintf(stderr, "To be implemented\n");
		KaBot *bot = (KaBot*)robot_base;
//		bot->moveForward(forward);
//		bot->moveRight(turn);
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		// expects left, right
		bot->Move(forward, turn);
//		bot->moveForward(forward);
//		bot->moveRight(turn);
		break;
	}
	default:
		fprintf(stderr, "There is no way to drive a robot without knowing its layout\n");
		break;
	}
}

//! Halt does not set last command, so go() can be used to continue
void CMotors::halt() {
	robot_base->EnableMotors(false);
}

void CMotors::go() {
	robot_base->EnableMotors(true);
}
