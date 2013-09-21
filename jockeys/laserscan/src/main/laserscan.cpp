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
#include <sys/syslog.h>

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

	std::string cam_port = "10002";
	if (argc >= 3) {
		cam_port = std::string(argv[2]);
	}
	std::cout << "Streaming images will be on port " << cam_port << " on receiving MSG_CAM_VIDEOSTREAM_START" << std::endl;

	if (argc >= 4) {
		std::cerr << "Too many arguments" << std::endl;
	}
	// temporary data structures, so we do not allocate memory all the time
	MotorCommand motorCommand;
	MappedObjectPosition positionForMappedObject;

	CMessage message;
	bool quitController = false;
	bool runController = false;
	while (!quitController){
		message = controller.getMessage();

		if (message.type != MSG_NONE) {
//			std::cout << "******************************************************************************" << std::endl;
			std::cout << "Command: " << message.getStrType();
//			if (message.len) {
//				std::cout << " with payload of length " << message.len << ", namely: ";
//				for (int i = 0; i < message.len; i++) {
//					std::cout << (int)message.data[i] << ',';
//				}
//			}
			std::cout << std::endl;
//			std::cout << "******************************************************************************" << std::endl;
		}

		if (gStop) {
			message.type = MSG_QUIT;
		}

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
			std::cout << "Started controller" << std::endl;
			break;
		}
		case MSG_CAM_VIDEOSTREAM_START: { // you also have to call MSG_START
//			runController = true;
//			controller.start();
//			controller.setVerbosity(LOG_DEBUG);
			controller.startVideoStream(cam_port);
			controller.acknowledge();
//			controller.setVerbosity(LOG_EMERG);
			break;
		}
		case MSG_CAM_VIDEOSTREAM_STOP: {
			controller.stopVideoStream();
			controller.acknowledge();
			break;
		}
		case MSG_SPEED: {
			memcpy(&motorCommand, message.data, sizeof(MotorCommand));
			controller.motorCommand(motorCommand);
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
			std::cout << "Detect step with the laser" << std::endl;
			int len = sizeof(struct MappedObjectPosition);
			if (message.len != len) {
				std::cerr << "Error, expected payload of MappedObjectPosition of size " << len << " while it is " << message.len << std::endl;
			}
			memcpy(&positionForMappedObject, message.data, message.len);
			controller.sendDetectedObject(positionForMappedObject);
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

