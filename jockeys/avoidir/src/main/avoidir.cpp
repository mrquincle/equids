/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief User leds to avoid all collisions
 * @file avoidall.cpp
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

#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <iomanip>

/***********************************************************************************************************************
 * Middleware includes
 **********************************************************************************************************************/

#include <IRobot.h>

/***********************************************************************************************************************
 * Jockey framework includes
 **********************************************************************************************************************/

#include <CMotors.h>
#include <CLeds.h>

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "AvoidInfrared";

//! Global stop condition
bool gStop = false;

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs. To
 * really quit the user is explained he/she will need to press Ctrl+\.
 */
void sigproc(int) {
	if (!gStop) {
		gStop = true;
		std::cout << "You used Ctrl+c to quit. We will gracefully end. User Ctrl+\\ if you want to end directly."
				<< std::endl;
	}
}

/**
 * Show that the controller ended properly by a LED show.
 */
void signal_end(CLeds &leds) {
	leds.color(LC_ORANGE);
	sleep(1);
	leds.color(LC_GREEN);
}

/**
 * Quit the controller indeed. So, not only set the speed to zero, but also halt the motors (and turn them off).
 */
void graceful_end(CMotors & motors) {
	motors.setSpeeds(0, 0);
	sleep(1);
	motors.halt();
	sleep(1);

	std::cout << "Avoidance controller quits" << std::endl;
	exit(EXIT_SUCCESS);
}

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
	int nof_switches = 2;
	bool calibrate = false;
	bool print = false;
	int timespan = 1000;
	bool forever = true;

	signal(SIGINT, sigproc);

	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;

	if (argc > 1) {
		std::string arg1 = std::string(argv[1]);
		if (arg1.find("calibrate") != std::string::npos) {
			calibrate = true;
		}
	}

	RobotBase::RobotType robot_type = RobotBase::Initialize(NAME);
	RobotBase* robot = RobotBase::Instance();
	for (int i = 0; i < 4; ++i)
		robot->SetPrintEnabled(i, false);

	//	std::cout << "Reset robot manually" << std::endl;
	//	RobotBase::MSPReset();

	std::cout << "Initialised robot of type " << RobotTypeStr[robot_type] << std::endl;
	//	IRobotFactory factory;
	//	RobotBase* robot = factory.GetRobot();
	//	RobotBase::RobotType robot_type = factory.GetType();

	// we need to initialize the motors before calibrate leds (which turns the robot around)
	CMotors motors(robot, robot_type);
	motors.init();

	std::cout << "Setup leds functionality" << std::endl;
	CLeds leds(robot, robot_type);
	leds.init();

	if (calibrate) {
		std::cout << "Calibrate!" << std::endl;
		leds.calibrate();
		std::cout << "Calibration done" << std::endl;
		graceful_end(motors);
	} else {
		std::cout << "Get calibration values" << std::endl;
		leds.get_calibration();
	}

	if (print) {
		int t = 0;
		do {
			std::cout << '[' << std::setw(4) << std::setfill('0') << t << "]: ";
			for (int i = 0; i < 8; ++i)
				std::cout << leds.distance(i) << ' ';
			std::cout << std::endl;
			usleep(100000);
		} while (forever || (++t != timespan));
		graceful_end(motors);
	}

	std::cout << "Sliding window size used of " << leds.get_window_size() << std::endl;

	int t = 0;
	do {
		int speed = 40;
		int radius = 0;
		leds.direction(speed, radius);
		for (int s = 0; s < leds.get_window_size(); ++s)  {
			for (int i = 0; i < 8; ++i) leds.distance(i);
			usleep(1000); // 1000 * 100 is every 0.1seconds
		}
		//std::cout << "Send wheel commands [" << speed << ',' << radius << ']' << std::endl;
		motors.setSpeeds(speed, radius);
		//usleep(1000000);
		//sleep(2);
	} while (!gStop && (forever || (++t != timespan)));

	signal_end(leds);
	graceful_end(motors);
	return EXIT_SUCCESS;
}
