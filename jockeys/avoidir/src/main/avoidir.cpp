/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief User infrared to avoid all collisions
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
#include <CInfrared.h>

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "AvoidInfraRed";

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(0);
	}
}

void graceful_end(CMotors & motors) {
	motors.setSpeeds(0, 0);
	usleep(1000);
	motors.halt();
	usleep(1000);

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

	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

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

	// we need to initialize the motors before calibrate infrared (which turns the robot around)
	CMotors motors(robot, robot_type);
	motors.init();

	std::cout << "Setup infrared functionality" << std::endl;
	CInfrared infrared(robot, robot_type);
	infrared.init();

	if (calibrate) {
		std::cout << "Calibrate!" << std::endl;
		infrared.calibrate();
		std::cout << "Calibration done" << std::endl;
		graceful_end(motors);
	} else {
		std::cout << "Get calibration values" << std::endl;
		infrared.get_calibration();
	}

	if (print) {
		for (int t = 0; t < timespan; ++t)  {
			std::cout << '[' << std::setw(4) << std::setfill('0') << t << "]: ";
			for (int i = 0; i < 8; ++i)
				std::cout << infrared.distance(i) << ' ';
			std::cout << std::endl;
			usleep(100000);
		}
		graceful_end(motors);
	}

	std::cout << "Sliding window size used of " << infrared.get_window_size() << std::endl;


	for (int t = 0; t < timespan; ++t)  {
		int speed = 40;
		int radius = 0;
		infrared.direction(speed, radius);
		for (int s = 0; s < infrared.get_window_size(); ++s)  {
			for (int i = 0; i < 8; ++i) infrared.distance(i);
			usleep(1000); // 1000 * 100 is every 0.1seconds
		}
		std::cout << "Send wheel commands [" << speed << ',' << radius << ']' << std::endl;
		motors.setSpeeds(speed, radius);
		//usleep(1000000);
		//sleep(2);
	}

	graceful_end(motors);
	return EXIT_SUCCESS;
}


