/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Back and forth controller
 * @file backandforth.cpp
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
 * @case      Testing
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

//#include <comm/IRComm.h>

/***********************************************************************************************************************
 * Jockey framework includes
 **********************************************************************************************************************/

#include "../eth/CMessage.h"
#include "../eth/CMessageClient.h"

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "ActionSelection";

#define DEBUG \
	NAME << '[' << getpid() << "] " << __func__ << "(): "

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(EXIT_SUCCESS);
	}
}

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	std::string port0, port1;
	if (argc > 2) {
		port0 = std::string(argv[1]);
		port1 = std::string(argv[2]);
	} else {
		std::cout << "Usage: actionselection port0 port1" << std::endl;
		return EXIT_FAILURE;
	}

	RobotBase* robot = RobotBase::Instance();
	RobotBase::RobotType robot_type = RobotBase::Initialize(NAME);

//	IRobotFactory factory;
//	RobotBase* robot = factory.GetRobot();
//	RobotBase::RobotType robot_type = factory.GetType();

	switch(robot_type) {
	case RobotBase::UNKNOWN: std::cout << "Detected unknown robot" << std::endl; break;
	case RobotBase::KABOT: std::cout << "Detected Karlsruhe robot" << std::endl; break;
	case RobotBase::ACTIVEWHEEL: std::cout << "Detected Active Wheel robot" << std::endl; break;
	case RobotBase::SCOUTBOT: std::cout << "Detected Scout robot" << std::endl; break;
	default:
		std::cout << "No known type (even not unknown). Did initialization go well?" << std::endl;
	}

	CMessageClient client[2];
	std::string host = "127.0.0.1";
	bool requirements[] = {true,false,false,false,false};
	std::cout << "Create (receiving) client on port " << port0 << std::endl;
	client[0].init(host.c_str(), port0.c_str(), requirements);
	std::cout << "Create (receiving) client on port " << port1 << std::endl;
	client[1].init(host.c_str(), port1.c_str(), requirements);

	CMessage message;
	for (int i = 0; i < 10; ++i) {
		int index = i % 2;
		int other = 1 - index;
		message.type = MSG_STOP;
		std::cout << DEBUG << "Stop controller " << index << std::endl;
		client[index].sendMessage(&message);
		sleep(1);

		message.type = MSG_START;
		std::cout << DEBUG << "Start controller " << other << std::endl;
		client[other].sendMessage(&message);
		sleep(1);

		message.type = MSG_SPEED;
		message.value1 = 50;
		message.value2 = -50;
		message.value3 = 0;

		std::cout << DEBUG << "Send controller 0 speed value " << message.value1 << ',' << message.value2 << std::endl;
		client[0].sendMessage(&message);
		sleep(1);

		message.value1 = -50;
		message.value2 = 50;
		message.value3 = 0;

		std::cout << DEBUG << ": Send controller 1 speed value " << message.value1 << ',' << message.value2 << std::endl;
		client[1].sendMessage(&message);
		sleep(1);
	}
	message.type = MSG_SPEED;
	message.value1 = 0;
	message.value2 = 0;
	client[0].sendMessage(&message);
	client[1].sendMessage(&message);
	message.type = MSG_QUIT;
	client[0].sendMessage(&message);
	client[1].sendMessage(&message);

	printf("Stopping %s\n", NAME.c_str());
	return 0;
}


