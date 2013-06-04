/**
  * @brief Control the laser (actually an actuator, not a sensor)
 * @file CLaser.cpp
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Jul 30, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


// General files
#include <stdio.h>

// Plugin files
#include <CLaser.h>


/* **************************************************************************************
 * Implementation of CLaser
 * **************************************************************************************/

CLaser::CLaser(RobotBase *robot_base, RobotBase::RobotType robot_type) {
	printf("Create laser object\n");
	if (robot_base == NULL) {
		fprintf(stderr, "robot_base is null, error in instantiation!\n");
	}
	this->robot_base = robot_base;
	this->robot_type = robot_type;
	status_on = false;
}

CLaser::~CLaser() {

}

/**
 * For now this just returns the status within this class. It does not actually query the
 * microcontroller which is connected to the laser.
 */
bool CLaser::IsOn() {
	return status_on;
}

/**
 * Turn on the laser.
 */
void CLaser::On() {
	status_on = true;
	switch (robot_type) {
//	case RobotBase::ACTIVEWHEEL: {
//		ActiveWheel *bot = (ActiveWheel*)robot_base;
//		break;
//	}
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot_base;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::SCOUTBOT: {
		printf("%s: For scout\n", __func__);
		ScoutBot *bot = (ScoutBot*)robot_base;
		bot->activateLaser(status_on);
		bot->sides[3]->led.all(LED_RED);
		bot->sides[3]->led.single((LEDPosition)(3), LED_MAGENTA);
		break;
	}
	default:
		fprintf(stderr, "I don't know how to turn on the laser on this robot type\n");
		break;
	}
}

void CLaser::Off() {
	status_on = false;
	switch (robot_type) {
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot_base;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::SCOUTBOT: {
		printf("%s: For scout\n", __func__);
		ScoutBot *bot = (ScoutBot*)robot_base;
		bot->activateLaser(status_on);
		bot->sides[3]->led.all(LED_WHITE);
		bot->sides[3]->led.single((LEDPosition)(3), LED_WHITE);
		break;
	}
	default:
		fprintf(stderr, "I don't know how to turn off the laser on this robot type\n");
		break;
	}
}

void CLaser::Toggle() {
	status_on = !status_on;
	switch (robot_type) {
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot_base;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::SCOUTBOT: {
		printf("%s: For scout\n", __func__);
		ScoutBot *bot = (ScoutBot*)robot_base;
		bot->activateLaser(status_on);
		break;
	}
	default:
		fprintf(stderr, "I don't know how to toggle the laser on this robot type\n");
		break;
	}
}
