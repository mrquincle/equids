/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Get a laser scan from the combination of laser and camera
 * @file laserscan.cpp
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

#include <signal.h>

/***********************************************************************************************************************
 * Controller include
 **********************************************************************************************************************/

#include <LaserScanController.h>

/***********************************************************************************************************************
 * Most important configuration parameters
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "LaserScan";

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

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
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {

	signal(SIGINT, sigproc);

	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;

	LaserScanController controller;
	controller.parsePort(argc, argv);
	controller.initServer();

	CMessage message;
	bool quitController = false;
	bool runController = false;
	while (!quitController){
		message = controller.getMessage();

		if (message.type != MSG_NONE)
			std::cout << "Command: " << message.getStrType() << " of length " << message.len << std::endl;

		switch (message.type) {
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
		case MSG_LASER_DETECT_STEP: {
			controller.sendDetectedObject();
			break;
		}
		case MSG_QUIT: {
			runController = true;
			controller.pause();
			controller.acknowledge();
			quitController = true;
			break;
		}

		}
	}
	return EXIT_SUCCESS;
}

