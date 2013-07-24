/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Establishing position and communicating it with others
 * @file position.cpp
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

/***********************************************************************************************************************
 * Jockey framework includes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Additional library includes
 **********************************************************************************************************************/

//#include <wapi/wapi.h>

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "Position";

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(0);
	}
}

//using namespace wapi;

#define JOIN_CHANNEL 55

void safe_close() {
	// flush, because deallocation can go wrong somewhere and we'd have a memory dump
	std::cout << std::endl << flush;
	sleep(1);
	printf("Robot object is automatically deleted by the factory.\n");
}

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
	int nof_switches = 10;

	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	IRobotFactory factory;
	RobotBase* robot = factory.GetRobot();
	RobotBase::RobotType robot_type = factory.GetType();

//	WAPI wapi(0);
//	int wapi_error = wapi.join(JOIN_CHANNEL);
//	if(WAPI::WAPI_OK != wapi_error)
//	{
//		std::cout << "Cannot join to channel " << JOIN_CHANNEL << " wapi_error " <<
//				wapi_error << endl;
//		return (EXIT_FAILURE);
//	}
	std::cout << "Joined to channel " << JOIN_CHANNEL << endl;

	safe_close();
	return 0;
}


