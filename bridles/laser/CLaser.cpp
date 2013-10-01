/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Control the laser (which is an actuator, not a sensor)
 * @file CLaser.cpp
 *
 * This file is created at Almende B.V. and Distributed Organisms B.V. It is open-source software and belongs to a
 * larger suite of software that is meant for research on self-organization principles and multi-agent systems where
 * learning algorithms are an important aspect.
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * Copyright (c) 2013 Anne C. van Rossum <anne@almende.org>
 *
 * @author    Anne C. van Rossum
 * @date      Jul 30, 2012
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

// General files
#include <stdio.h>

// Plugin files
#include <CLaser.h>


/* **************************************************************************************
 * Implementation of CLaser
 * **************************************************************************************/

CLaser::CLaser(RobotBase *robot_base, RobotBase::RobotType robot_type) {
//	printf("Create laser object\n");
	log_prefix = "CLamera: ";
	if (robot_base == NULL) {
		std::cerr << log_prefix << "robot_base is null, error in instantiation!" << std::endl;
	}
	this->robot = robot_base;
	this->robot_type = robot_type;
	status_on = false;
}

CLaser::~CLaser() {
	robot = NULL;
	status_on = false;
	std::cout << log_prefix << "Laser object deallocated" << std::endl;
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
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::SCOUTBOT: {
//		printf("%s: For scout\n", __func__);
		ScoutBot *bot = (ScoutBot*)robot;
		bot->activateLaser(status_on);
//		robot->SetLEDAll(bot->GetSide(RobotBase::FRONT), LED_RED);
		robot->SetLEDAll(bot->GetSide(RobotBase::FRONT), LED_OFF);
		break;
	}
	case RobotBase::ACTIVEWHEEL: default:
		std::cerr << log_prefix << "I don't know how to turn on the laser on this robot type (there is no laser)" << std::endl;
		break;
	}
}

void CLaser::Off() {
	status_on = false;
	switch (robot_type) {
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::SCOUTBOT: {
//		printf("%s: For scout\n", __func__);
		ScoutBot *bot = (ScoutBot*)robot;
		bot->activateLaser(status_on);
		robot->SetLEDAll(bot->GetSide(RobotBase::FRONT), LED_OFF);
		break;
	}
	case RobotBase::ACTIVEWHEEL: default:
		std::cerr << log_prefix << "I don't know how to turn off the laser on this robot type (there is no laser)" << std::endl;
		break;
	}
}

void CLaser::Toggle() {
	status_on = !status_on;
	switch (robot_type) {
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::SCOUTBOT: {
//		printf("%s: For scout\n", __func__);
		ScoutBot *bot = (ScoutBot*)robot;
		bot->activateLaser(status_on);
		break;
	}
	case RobotBase::ACTIVEWHEEL: default:
		std::cerr << log_prefix << "I don't know how to toggle the laser on this robot type (there is no laser)" << std::endl;
		break;
	}
}
