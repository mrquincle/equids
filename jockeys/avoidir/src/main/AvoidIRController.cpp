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

AvoidIRController::AvoidIRController(): motors(NULL), leds(NULL), collision(false) {

}

AvoidIRController::~AvoidIRController() {

}

void AvoidIRController::initRobotPeriphery() {
	// we need to initialize the motors before calibrate leds (which turns the robot around)
	motors = new CMotors(robot, robot_type);
	motors->init();

	if (log_level >= LOG_INFO) {
		std::cout << "Setup leds functionality" << std::endl;
	}
	leds = new CLeds(robot, robot_type);
	bool success = leds->init();

	if (!success) {
		if (log_level >= LOG_EMERG) {
			std::cerr << "Error with LED initialization" << std::endl;
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

	leds->collision();
	usleep(100000);
	return;

//	leds->update();

	if (leds->encounter()) {
		std::cout << std::endl << "Encountered a robot" << std::endl;
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
	if (log_level >= LOG_INFO) {
		std::cout << "Rotate for about 90 degrees" << std::endl;
	}
	motors->rotate(90);
	motors->set_to_zero();
	// reset histogram values in leds
	leds->reset();
	collision = false;
}

void AvoidIRController::head_back() {
	if (log_level >= LOG_INFO) {
		std::cout << "Go back for a few seconds" << std::endl;
	}
#ifdef USE_CVUT
	int speed = 30; int turn = 0;
	motors->setSpeeds(-speed, turn);
#else
	int speed = 30; int radius = 1000;
	motors->setRadianSpeeds(-speed, radius);
#endif
	sleep(2);
	motors->set_to_zero();
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
	if (leds->collision()) {
		collision = true;
		reportCollision();
		if (standalone) {
			head_back();
			escape();
		} else {
			head_back();
			motors->set_to_zero();
		}
	}

	// by default straight forward
	// just go forward
	if (!collision) {
//#define USE_CVUT
#ifdef USE_CVUT
		int speed = 30; int turn = 0;
		if (log_level >= LOG_INFO) {
			std::cout << "Send wheel commands [" << speed << ',' << turn << ']' << std::endl;
		}
		motors->setSpeeds(speed, turn);
		sleep(2);
		motors->setSpeeds(0, 0);
#else
		int speed = 30; int radius = 1000;
		if (log_level >= LOG_INFO) {
			std::cout << "Send wheel commands [" << speed << ',' << radius << ']' << std::endl;
		}
		motors->setRadianSpeeds(speed, radius);
		sleep(2);
		motors->set_to_zero();
#endif
	}
}

void AvoidIRController::calibrate() {
	if (log_level >= LOG_INFO) {
		std::cout << "Calibrate!" << std::endl;
	}
	leds->calibrate();
	if (log_level >= LOG_INFO) {
		std::cout << "Calibration done" << std::endl;
	}
	graceful_end();
}

void AvoidIRController::get_calibration() {
	if (log_level >= LOG_INFO) {
		std::cout << "Get calibration values" << std::endl;
	}
	leds->get_calibration();
	if (log_level >= LOG_INFO) {
		std::cout << "Sliding window size used of " << leds->get_window_size() << std::endl;
	}
}

void AvoidIRController::stop_motors() {
	std::cout << "Stop motors " << std::endl;
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

	std::cout << NAME << " quits" << std::endl;
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

