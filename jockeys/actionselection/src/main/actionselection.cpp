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

#include "CEquids.h"
#include "CMessage.h"

#include <BackandforthScenario.h>

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "ActionSelection";

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
 * Add to this list of scenarios for your own. In the end we want to use the scenario S_GRAND_CHALLENGE1.
 */
enum Scenario { SC_BACK_AND_FORTH_TEST, SC_STANDALONE_CAMERA, SC_GRAND_CHALLENGE1, SCENARIO_COUNT };

/**
 * Cast Scenario to a string. Usage: ScenarioStr[scenario].
 */
#define MACROSTR(k) #k
static const std::string ScenarioStr[] = {
		MACROSTR(SC_BACK_AND_FORTH_TEST), MACROSTR(SC_STANDALONE_CAMERA), MACROSTR(SC_GRAND_CHALLENGE1)
};
#undef MACROSTR

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;

	Scenario scenario = SC_GRAND_CHALLENGE1;
	std::cout << "We will use scenario " << ScenarioStr[scenario] << std::endl;

	CEquids equids;
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	if (argc > 1) {
		std::cout << "Reading init" << std::endl;
		if (!equids.init(argv[1])) {
			std::cout << "Wrong configuration file " << argv[1] << std::endl;
			return EXIT_FAILURE;
		}
		std::cout << "Finish init" << std::endl;
	} else {
		std::cout << "Usage: actionselection jockey_cfg" << std::endl;
		return EXIT_FAILURE;
	}

	bool quit = false;

	switch (scenario) {
	case SC_BACK_AND_FORTH_TEST: {
		BackandforthScenario scenario(&equids);
		if (!scenario.Init()) {
			std::cerr << "Error in initialization, break out" << std::endl;
			break;
		}
		scenario.Run();
		break;
	}
	case SC_GRAND_CHALLENGE1:
		std::cerr << "Not implemented yet!" << std::endl;
		break;
	case SC_STANDALONE_CAMERA:
//		equids.initJockey(camera);
		break;
	default:
		std::cerr << "Unknown scenario" << std::endl;
	}

	std::cout << "Do quit controllers" << std::endl;
	equids.quit();
	return 0;
}


