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
#include <math.h>
#include <iomanip>

/***********************************************************************************************************************
 * Controller include
 **********************************************************************************************************************/

#include <LaserScanController.h>

/***********************************************************************************************************************
 * Most important configuration parameters
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
static const std::string NAME = "LaserScan";

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

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

static inline float getPhi(struct UbiPosition pos1, struct UbiPosition pos2) {
	float phi = atan2( pos2.y - pos1.y,  pos2.x - pos1.x );
	return phi;
}

static inline void print(const MappedObjectPosition& p, std::string extra_text="") {
    std::cout << DEBUG << extra_text << std::fixed << std::setw( 9 ) << std::setprecision( 7 ) <<
    		"position [ " << p.xPosition << ',' << p.yPosition << ',' << p.zPosition << ',' << p.phiPosition <<
    		" ] with uncertainty: [" <<
    		p.xUncertainty << ',' << p.yUncertainty << ',' << p.zUncertainty << ',' << p.phiUncertainty << " ]" << std::endl;
}

static inline void print(const UbiPosition& p, std::string extra_text="") {
    std::cout << DEBUG << extra_text << std::fixed << std::setw( 9 ) << std::setprecision( 7 ) <<
    		"position [ " << p.x << ',' << p.y << ',' << p.z << "] at t=" << p.time_stamp << std::endl;
}

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {

	signal(SIGINT, sigproc);

	std::cout << "################################################################################" << std::endl;
	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;
	std::cout << "################################################################################" << std::endl;

	LaserScanController controller;
	controller.parsePort(argc, argv);
	controller.initServer();

	std::string cam_port = "10002";
	if (argc >= 3) {
		cam_port = std::string(argv[2]);
	}
	std::cout << DEBUG << "Streaming images will be on port " << cam_port << " on receiving MSG_CAM_VIDEOSTREAM_START" << std::endl;

	bool enable_option = false;
	if (argc == 4) {
		std::string arg4 = std::string(argv[3]);
		if (arg4.find("standalone") != std::string::npos) {
			enable_option = true;
		}
	}

	if (argc >= 5) {
		std::cerr << "Too many arguments" << std::endl;
	}
	// temporary data structures, so we do not allocate memory all the time
	MotorCommand motorCommand;
	MappedObjectPosition positionForMappedObject;

	CMessage message;
	bool quitController = false;
	bool runController = false;

	UbiPosition position_before;
	UbiPosition position_after;
	UbiPosition temp;
	ObjectType object_type;

	int acquisition_position_count = 2; // should be > 0

	int get_position_before = 0;
	int get_position_after = 0;

	while (!quitController){

		usleep(50000);
		if (!runController)
			usleep(450000);

		message = controller.getMessage();

		if (message.type != MSG_NONE) {
			std::cout << DEBUG << "Command: " << message.getStrType() << std::endl;
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
			std::cout << DEBUG << "Started controller" << std::endl;
			break;
		}
		case MSG_CAM_TURN_ON_ALL_THE_TIME: {
			bool value = false;
			if (message.len == 1) {
				uint8_t val = message.data[0];
				value = (bool)val;
			} else if (message.len == 0) {
				value = true;
			}
			std::cout << DEBUG << "Set camera to exclusive use to this controller" << std::endl;
			controller.setCameraExclusive(value);
			controller.startVideoStream(cam_port);
			controller.acknowledge();
			break;
		}
		case MSG_CAM_VIDEOSTREAM_START: { // you also have to call MSG_START
			std::cout << DEBUG << "Received message video start" << std::endl;
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
		case MSG_UBISENCE_POSITION: {
			// second time
			if (get_position_after) {
				get_position_after--;
				int len = sizeof(struct UbiPosition);
				if (message.len != len) {
					std::cerr << DEBUG << "Error, expected payload of UbiPosition of size " << len << " while it is " << message.len << std::endl;
				}
				if (get_position_after == acquisition_position_count-1) {
					memcpy(&position_after, message.data, message.len);
				} else {
					memcpy(&temp, message.data, message.len);
					position_after.x = (position_after.x + temp.x);
					position_after.y = (position_after.y + temp.y);
					position_after.z = (position_after.z + temp.z);
				}

				std::cout << DEBUG << "Got position after moving backwards: " << get_position_after << std::endl;
				if (!get_position_after) {
					position_after.x = position_after.x / acquisition_position_count;
					position_after.y = position_after.y / acquisition_position_count;
					position_after.z = position_after.z / acquisition_position_count;
					MappedObjectPosition positionForMappedObject;
					// store the object on the position before(!)
					positionForMappedObject.xPosition = position_before.x;
					positionForMappedObject.yPosition = position_before.y;
					positionForMappedObject.zPosition = position_before.z;
					positionForMappedObject.phiPosition = getPhi(position_before, position_after);
					controller.sendDetectedObject(object_type, positionForMappedObject);
					print(positionForMappedObject, "Object ");
					controller.calcDistance(true);
				}
			}

			// first time
			if (get_position_before) {
				get_position_before--;
				int len = sizeof(struct UbiPosition);
				if (message.len != len) {
					std::cerr << DEBUG << "Error, expected payload of UbiPosition of size " << len << " while it is " << message.len << std::endl;
				}
				std::ostringstream msg; msg.clear(); msg.str(""); msg << (acquisition_position_count - get_position_before) << ' ';
				std::string nr = msg.str();
				if (get_position_before == acquisition_position_count-1) {
					memcpy(&position_before, message.data, message.len);
					print(position_before,nr);
				} else {
					memcpy(&temp, message.data, message.len);
					print(temp,nr);
					position_before.x = (position_before.x + temp.x);
					position_before.y = (position_before.y + temp.y);
					position_before.z = (position_before.z + temp.z);
				}

				std::cout << DEBUG << "Got position before moving backwards: " << get_position_before << std::endl;
				if (!get_position_before) {
					position_before.x = position_before.x / acquisition_position_count;
					position_before.y = position_before.y / acquisition_position_count;
					position_before.z = position_before.z / acquisition_position_count;
					get_position_after = acquisition_position_count;
					print(position_before, "Before ");
					int distance = 0;
					bool success = controller.getDistance(distance);
					bool good_position = (success && (distance > 10) && (distance < 30));
					if (good_position) {
						std::cout << DEBUG << "Distance is already a nice 10-30cm, no need to back up for better position" << std::endl;
					} else {
						std::cout << DEBUG << "Move back a bit to get better position for laser" << std::endl;
						controller.head_back();
					}
					object_type = controller.getDetectedObject();
					std::cout << DEBUG << "Move back more for better Ubisense positioning" << std::endl;
					if (good_position) {
						// not backed up before, so do it a bit more now, maybe better controller.head_back(1) two times
						controller.head_back(2);
					} else {
						controller.head_back();
					}
				}
			}
			break;
		}
		case MSG_LASER_DETECT_STEP: {
			controller.calcDistance(false);
			get_position_before = acquisition_position_count;
			get_position_after = 0;
			std::cout << DEBUG << "Detect step with the laser" << std::endl;
			std::cout << DEBUG << "Waiting for redirected ZigBee messages over ethernet" << std::endl;
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

