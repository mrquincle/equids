/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief The total scenario of Grand Challenge 1
 * @file GrandChallenge1Scenario.cpp
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
 * @date      Aug 16, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Action selection
 */

#include <GrandChallenge1Scenario.h>

#include <iostream>

GrandChallenge1Scenario::GrandChallenge1Scenario(CEquids * equids) :
CScenario(equids) {
	J_MAPPING = J_LASER_RECOGNITION = J_VISUAL_EXPLORATION = J_INFRARED_EXPLORATION = J_WENGUO = -1;

	quit = false;

	// set initial state
	state = S_START;
}

GrandChallenge1Scenario::~GrandChallenge1Scenario() {

}

bool GrandChallenge1Scenario::Init() {
	bool continue_program = true;

	J_MAPPING = equids->find("mapping");
	//	J_WENGUO = equids->find("wenguo");
	J_LASER_RECOGNITION = equids->find("laser");
	J_INFRARED_EXPLORATION = equids->find("avoidir");

	//! Send an error message or also quit program..
	if (J_MAPPING == -1) {
		std::cout << "Not defined jockey \"mapping\"" << std::endl;
		continue_program = true;
	}
	//	if (J_WENGUO == -1) {
	//		std::cout << "Not defined jockey \"wenguo\"" << std::endl;
	//		continue_program = true;
	//	}
	if (J_LASER_RECOGNITION == -1) {
		std::cout << "Not defined jockey \"laser\"" << std::endl;
		continue_program = false;
	}

	if (J_INFRARED_EXPLORATION == -1) {
		std::cout << "Not defined jockey \"avoidir\"" << std::endl;
		continue_program = false;
	}

	//equids->initJockey(J_MAPPING);
	equids->initJockey(J_INFRARED_EXPLORATION);

	return continue_program;
}

void GrandChallenge1Scenario::Run() {

	while (!quit) {
		switch (state) {
		case S_START:
			//equids->switchToJockey(J_MAPPING);
			//state = S_MAPPING;
			// for now go directly to exploration, mapping has to be done later
			state = S_EXPLORATION;
			break;
		case S_MAPPING: {
			CMessage message = equids->getMessage(J_MAPPING);
			if (message.type == MSG_MAP_COMPLETE) {
				state = S_EXPLORATION;
				equids->switchToJockey(J_LASER_RECOGNITION);
			}
			break;
		}
		case S_EXPLORATION: {
			equids->switchToJockey(J_INFRARED_EXPLORATION);
			CMessage message = equids->getMessage(J_INFRARED_EXPLORATION);
			if (message.type == MSG_COLLISION_DETECTED) {
				std::cout << "Got message: " << StrMessage[message.type] << std::endl;
				state = S_DETECT_OBJECT;
			} else if (message.type != MSG_NONE){
				std::cout << "Didn't expect message " << StrMessage[message.type] << " from infrared " << std::endl;
			}
			//
			break;
		}
		case S_DETECT_OBJECT: {
			equids->switchToJockey(J_LASER_RECOGNITION);
			CMessage message = equids->getMessage(J_LASER_RECOGNITION);
			if (message.type == MSG_LASER_DETECT_STEP) {
				std::cout << "Got message: " << StrMessage[message.type] << std::endl;
				state = S_EXPLORATION;
				std::cout << "Go back to exploration" << std::endl;
			} else if (message.type != MSG_NONE){
				std::cout << "Didn't expect message " << StrMessage[message.type] << " from laser " << std::endl;
			}
			break;
		}
		case S_QUIT:
			quit = true;
			break;
		}
		usleep(100000);
	}
}



