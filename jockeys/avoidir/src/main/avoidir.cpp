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

#include <AvoidIRController.h>

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
 * Main routine for a collision avoidance controller that does only use infrared to perform this simple task. It can be
 * executed with "calibrate" as argument to perform calibration. During that phase it will rotate to find a smoothed
 * average to correct the bias of the different infrared leds. If a robot has been calibrated once, it can use the
 * calibration values because they are stored in a text file.
 */
int main(int argc, char **argv) {
	bool calibrate = false;
	bool standalone = false;

	signal(SIGINT, sigproc);

	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;

	AvoidIRController controller;
	controller.parsePort(argc, argv);

	if (argc == 3) {
		std::string arg2 = std::string(argv[2]);
		if (arg2.find("calibrate") != std::string::npos) {
			calibrate = true;
		} else if (arg2.find("standalone") != std::string::npos) {
			standalone = true;
		}
	} else if ((argc > 3) || (argc <= 1)) {
		std::cerr << DEBUG << " usage: " << argv[0] << " PORT [OPTION]" << std::endl << std::endl;
		std::cerr << "OPTION: calibrate|standalone" << std::endl;
		return EXIT_FAILURE;
	}

	if (standalone || calibrate) {
		int timespan = 1000;
		controller.initRobot();
		controller.initRobotPeriphery();
		controller.start();
		if (calibrate) {
			controller.calibrate();
		} else {
			controller.get_calibration();
			for (int t = 0; t < timespan; ++t) {
				controller.tick();
			}
		}
		controller.signal_end();
		controller.graceful_end();
		return EXIT_SUCCESS;
	}

	controller.initServer();

	CMessage message;
	bool quitController = false;
	bool runController = false;
	while (!quitController){
		message = controller.getMessage();

		if (message.type != MSG_NONE)
			std::cout << "Command: " << message.getStrType() << ' ' << message.value1 << ',' \
			<< message.value2 << std::endl;

		switch (message.type){
		case MSG_INIT: {
			controller.initRobot();
			controller.initRobotPeriphery();
			controller.pause();
			controller.acknowledge();
			break;
		}
		case MSG_START: {
			runController = true;
			controller.start();
			controller.get_calibration();
			controller.acknowledge();
			break;
		}
		// do calibration - if you want to - after MSG_INIT or after MSG_STOP
		case MSG_CALIBRATE: {
			controller.start();
			controller.calibrate();
			controller.pause();
			controller.acknowledge();
			break;
		}
		case MSG_STOP: {
			runController = false;
			controller.pause();
			controller.acknowledge();
			break;
		}
		case MSG_NONE: {
			if (runController) {
				// do our thing
				controller.tick();
			}
			break;
		}
		case MSG_QUIT: {
			runController = true;
			controller.pause();
			controller.signal_end();
			controller.graceful_end();
			controller.acknowledge();
			quitController = true;
			break;
		}

		}
	}
	return EXIT_SUCCESS;
}
