/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CController.cpp
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
 * @date      Sep 2, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#include <CController.h>

#include <syslog.h> // LOG_EMERG

//! The name of the controller can be used for controller selection
static const std::string NAME = "CController";

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

CController::CController(): port(""), server(NULL), robot(NULL), robot_type(RobotBase::UNKNOWN), robot_id(-1),
log_level(LOG_EMERG) {}

CController::~CController() { }

//! First get the port of the jockey
void CController::parsePort(int argc, char **argv) {
	if (argc <= 1) {
		std::cout << DEBUG << "First parameter must be the port the jockey can be reached on" << std::endl;
		exit(EXIT_FAILURE);
	}
	port = std::string(argv[1]);
}

//! Create server and start it
void CController::initServer() {
	std::cout << "Create (receiving) message server on port " << port << std::endl;
	server = new CMessageServer();
	server->initServer(port.c_str());
}

void CController::initRobot() {
	robot_type = RobotBase::Initialize(NAME);
	robot = RobotBase::Instance();
	for (int i = 0; i < 4; ++i)
		robot->SetPrintEnabled(i, false);
	std::cout << "Initialized robot of type " << RobotTypeStr[robot_type] << std::endl;

	char* robotID = getenv("sr_id");
	if (robotID != NULL)
		robot_id = atoi(robotID);
	std::cout << "Robot has id " << robotID << std::endl;
}

void CController::acknowledge() {
	server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
}

void CController::pause() {
	robot->pauseSPI(true);
	usleep(10000);
}

void CController::start() {
	robot->pauseSPI(false);
	usleep(10000);
}


