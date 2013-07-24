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

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
	int nof_switches = 2;

	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	RobotBase* robot = RobotBase::Instance();
	RobotBase::RobotType robot_type = RobotBase::Initialize(NAME);

//	IRobotFactory factory;
//	RobotBase* robot = factory.GetRobot();
//	RobotBase::RobotType robot_type = factory.GetType();

	for (int i = 0; i < 4; ++i)
		robot->SetPrintEnabled(i, false);

	// we need to initialize the motors before calibrate infrared (which turns the robot around)
	CMotors motors(robot, robot_type);
	motors.init();

	std::cout << "Setup infrared functionality" << std::endl;
	CInfrared infrared(robot, robot_type);
	infrared.calibrate();

	infrared.distance(0);

	motors.setSpeeds(100, 0);
	sleep(2);

	motors.setSpeeds(-100, 0);
	sleep(2);

	motors.setSpeeds(0, 100);
	sleep(2);

	motors.setSpeeds(0, -100);
	sleep(2);

	motors.setSpeeds(0, 0);
	motors.halt();

	fprintf(stdout,"Stopping avoidance.");
	return 0;
}


