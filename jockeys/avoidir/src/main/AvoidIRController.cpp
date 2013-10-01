/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file AvoidIRController.cpp
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
 * @date      Aug 16, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#include <AvoidIRController.h>

//! The name of the controller can be used for controller selection
static const std::string NAME = "AvoidInfrared";

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

//#define REACT_TO_LEFT_AND_RIGHT
// #define RANDOM_ESCAPE // too much back and forth rotations then...

AvoidIRController::AvoidIRController(): motors(NULL), leds(NULL), collision(false), standalone(false) {
#ifdef RANDOM_ESCAPE
	srand(time(NULL));
#endif
}

AvoidIRController::~AvoidIRController() {

}

void AvoidIRController::start() {
	std::cout << DEBUG << "Starting the infrared avoidance" << std::endl;
	if (robot_type == RobotBase::ACTIVEWHEEL) {
		robot->SetLEDAll(robot->GetSide(RobotBase::RIGHT), LED_GREEN);
	} else {
		robot->SetLEDOne(robot->GetSide(RobotBase::RIGHT), 0, LED_WHITE);
		robot->SetLEDOne(robot->GetSide(RobotBase::LEFT), 1, LED_GREEN);
	}
	bool success = leds->init();
	if (!success) {
		std::cerr << DEBUG << "Re-initialization failed" << std::endl;
	} else {
		std::cerr << DEBUG << "Re-initialization leds" << std::endl;
	}
	CController::start();
}

void AvoidIRController::initRobotPeriphery() {
	// we need to initialize the motors before calibrate leds (which turns the robot around)
	std::ostringstream msg;
	msg.clear(); msg.str(""); msg << NAME << '[' << getpid() << "] ";

	motors = new CMotors(robot, robot_type);
	motors->setLogPrefix(msg.str());
	motors->init();

	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Setup leds functionality" << std::endl;
	}
	leds = new CLeds(robot, robot_type);
	leds->setLogPrefix(msg.str());
	bool success = leds->init();
	if (!success) {
		if (log_level >= LOG_EMERG) {
			std::cerr << DEBUG << "Error with LED initialization" << std::endl;
		}
		signal_end();
		graceful_end();
		exit(EXIT_SUCCESS);
	}
}

void AvoidIRController::print() {
	// sample the LEDs for a while, each update() takes a bit of time, so the total span is around 0.1 seconds.
	leds->setVerbosity(LOG_ERR);
	for (int s = 0; s < 10; ++s)  {
		leds->update();
	}
	leds->setVerbosity(LOG_INFO);

	if (robot_type == RobotBase::ACTIVEWHEEL) {
		for (int i = 0; i < 4; i++) {
			LedLocation loc = (LedLocation)(i*2);
			leds->collision(loc, 0);
			usleep(100000);
		}
	}

	leds->collision();
	usleep(100000);
	return;

	//	leds->update();

	if (leds->encounter()) {
		std::cout << DEBUG << "Encountered a robot" << std::endl;
	} else {
		std::cout << ".";
	}
	if (!(leds->messages_sent % 50)) std::cout << std::endl;
}

void AvoidIRController::reportCollision() {
	// overwrite message type
	CMessage msg;
	msg.type = MSG_COLLISION_DETECTED;

	// overwrite sender id
	//	position.mappedBy = robot_id;

	// set payload to empty
	msg.len = 0;
	msg.data = NULL;

	// send message
	server->sendMessage(msg);
}

void AvoidIRController::escape() {
	int dir = 1;
#ifdef RANDOM_ESCAPE
	dir = ((rand() % 2)<<1) - 1;
#endif
	int deg = dir * 90;
	//	if (log_level >= LOG_INFO) {
	std::cout << DEBUG << "Rotate for about " << deg << " degrees to escape" << std::endl;
	//	}
	motors->rotate(deg);
	motors->set_to_zero();
	// reset histogram values in leds
	leds->reset();
	collision = false;
}

void AvoidIRController::head_back() {
	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Go back for a few seconds" << std::endl;
	}
	//#ifdef USE_CVUT
	if (robot_type == RobotBase::ACTIVEWHEEL) {
		int speed = 30; int turn = 0;
		motors->setSpeeds(-speed, turn);
	} else {
		//#else
		int speed = 30; int radius = 1000;
		motors->setRadianSpeeds(-speed, radius);
	}
	//#endif
	sleep(2);
	motors->set_to_zero();
	motors->setSpeeds(0,0);
}

/**
 * The main function of the controller. This one uses the collision() function of the Leds class to know if there has
 * been a collision. Then it reverses for a bit, rotates for 60 degrees and starts driving again. This is repeated
 * each time tick() is called.
 */
void AvoidIRController::tick() {
	// just return when there has been a collision, first call escape()
	if (collision) return;

	// sample the LEDs for a while, each update() takes a bit of time, so the total span is around 0.1 seconds.
	for (int s = 0; s < 40; ++s)  {
		leds->update();
	}

	// only on a collision, drive the wheels
	if (robot_type == RobotBase::SCOUTBOT) {
		if (leds->collision()) {
			collision = true;
			reportCollision();
			if (standalone) {
				std::cout << DEBUG << "In standalone setting, so head-back and escape" << std::endl;
				head_back();
				escape();
			} else {
				//			head_back();
				stop_motors();
			}
		}
	}

	//#ifdef REACT_TO_LEFT_AND_RIGHT
	if (robot_type == RobotBase::ACTIVEWHEEL) {
		int adjust_threshold = 0;
		switch (robot_type) {
		case RobotBase::SCOUTBOT:{
			adjust_threshold = 50;
		}
		break;
		case RobotBase::ACTIVEWHEEL:{
			adjust_threshold = 0;
		} break;
		default:
			break;
		}

		// for robot 71
		int adjust_threshold_right = adjust_threshold + 160;
		int adjust_threshold_left =  adjust_threshold + 160;

		// adjust treshold in general t 50
		if (leds->collision(LL_FRONT_RIGHT, 50)) {
			std::cout << DEBUG << "Collision at the front (big arm)" << std::endl;
			motors->rotate(40);
		}else if (leds->collision(LL_LEFT_FRONT, adjust_threshold_left)) {
			std::cout << DEBUG << "Collision at the left" << std::endl;
			motors->setSpeeds(-60, 0);
			sleep(2);
			//		motors->rotate(90);
			// or just a tiny bit while going forward
			motors->rotate(40);
//			motors->setSpeeds(40, -20);
		} else if (leds->collision(LL_RIGHT_REAR, adjust_threshold_right)) {
//			std::cout << DEBUG << "Collision at the right" << std::endl;
//			motors->setSpeeds(-60, 0);
//			sleep(2);
//			motors->rotate(-90);
			// or just a tiny bit while going forward
//			motors->setSpeeds(40, -20);
			motors->rotate(-40);

		} else if (leds->collision(LL_REAR_LEFT, adjust_threshold)) {
			std::cout << DEBUG << "Collision at the rear, just go on" << std::endl;
			motors->rotate(-40);
		}
	}
	//#endif
	// by default straight forward
	// just go forward
	if (!collision) {
		//#define USE_CVUT
#ifdef USE_CVUT
		int speed = 30; int turn = 0;
		if (log_level >= LOG_INFO) {
			std::cout << DEBUG << "Send wheel commands [" << speed << ',' << turn << ']' << std::endl;
		}
		motors->setSpeeds(speed, turn);
		sleep(2);
		motors->setSpeeds(0, 0);
#else
		if (robot_type == RobotBase::SCOUTBOT) {
			int speed = 15; int radius = 1000;
			if (log_level >= LOG_INFO) {
				std::cout << DEBUG << "Send wheel commands [" << speed << ',' << radius << ']' << std::endl;
			}
			motors->setRadianSpeeds(speed, radius);
		} else {
			motors->setSpeeds(40,0);
		}
		//		sleep(2);
		//		motors->set_to_zero();
#endif
	}
}

void AvoidIRController::calibrate() {
	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Calibrate!" << std::endl;
	}
	leds->calibrate();
	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Calibration done" << std::endl;
	}
	graceful_end();
}

void AvoidIRController::get_calibration() {
	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Get calibration values" << std::endl;
	}
	leds->get_calibration();
	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Sliding window size used of " << leds->get_window_size() << std::endl;
	}
}

void AvoidIRController::stop_motors() {
	std::cout << DEBUG << "Stop motors " << std::endl;
	motors->set_to_zero();
	motors->setSpeeds(0, 0);
	sleep(1);
}

/**
 * Set the speed to zero, but also halt the motors (and turn them off).
 */
void AvoidIRController::graceful_end() {
	motors->setSpeeds(0, 0);
	sleep(1);
	motors->halt();
	sleep(1);

	std::cout << DEBUG << "Graceful end..." << std::endl;
	//exit(EXIT_SUCCESS);
}

/**
 * Show that the controller ended properly by a LED show.
 */
void AvoidIRController::signal_end() {
	leds->color(LC_ORANGE);
	sleep(1);
	leds->color(LC_GREEN);
}

