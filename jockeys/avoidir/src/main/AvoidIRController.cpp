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

AvoidIRController::AvoidIRController(): motors(NULL), leds(NULL) {

}

AvoidIRController::~AvoidIRController() {

}

void AvoidIRController::initRobotPeriphery() {
	// we need to initialize the motors before calibrate leds (which turns the robot around)
	motors = new CMotors(robot, robot_type);
	motors->init();

	std::cout << "Setup leds functionality" << std::endl;
	leds = new CLeds(robot, robot_type);
	leds->init();
}

void AvoidIRController::print() {
	// sample the LEDs for a while, each update() takes a bit of time, so the total span is around 0.1 seconds.
//	leds->setVerbosity(LOG_ERR);
//	for (int s = 0; s < leds->get_window_size(); ++s)  {
//		leds->update();
//	}
//	leds->setVerbosity(LOG_INFO);
//	leds->update();

	if (leds->encounter()) {
		std::cout << std::endl << "Encountered a robot" << std::endl;
	} else {
		std::cout << ".";
	}
	if (!(leds->messages_sent % 50)) std::cout << std::endl;
}

/**
 * The main function of the controller. This one uses the collision() function of the Leds class to know if there has
 * been a collision. Then it reverses for a bit, rotates for 60 degrees and starts driving again. This is repeated
 * each time tick() is called.
 */
void AvoidIRController::tick() {
	// by default straight forward
	int speed = 40;
	int radius = 1000;

	// sample the LEDs for a while, each update() takes a bit of time, so the total span is around 0.1 seconds.
	for (int s = 0; s < leds->get_window_size(); ++s)  {
		leds->update();
	}

	// only on a collision, drive the wheels
	if (leds->collision()) {
		// two seconds in reverse
		std::cout << "Go back and rotate for 60 degrees" << std::endl;
		motors->setRadianSpeeds(-speed, radius);
		sleep(2);
		// and rotate
		motors->rotate(60);
	} else {
		// else just go forward
		motors->setRadianSpeeds(speed, radius);
	}

	if (log_level >= LOG_DEBUG) {
		std::cout << "Send wheel commands [" << speed << ',' << radius << ']' << std::endl;
	}
}

void AvoidIRController::calibrate() {
	std::cout << "Calibrate!" << std::endl;
	leds->calibrate();
	std::cout << "Calibration done" << std::endl;
	graceful_end();
}

void AvoidIRController::get_calibration() {
	std::cout << "Get calibration values" << std::endl;
	leds->get_calibration();
	std::cout << "Sliding window size used of " << leds->get_window_size() << std::endl;
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

