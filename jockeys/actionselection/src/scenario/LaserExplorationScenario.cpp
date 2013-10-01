/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief The total scenario of Grand Challenge 1
 * @file LaserExplorationScenario.cpp
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

#include <LaserExplorationScenario.h>

#include <iostream>

#include <messageDataType.h>

LaserExplorationScenario::LaserExplorationScenario(CEquids * equids): CScenario(equids) {
	J_LASER_RECOGNITION = J_POSITION = J_RANDOM_EXPLORATION = -1;

	quit = false;

	// set initial state
	state = S_START;
}

LaserExplorationScenario::~LaserExplorationScenario() {

}

bool LaserExplorationScenario::Init() {
	bool continue_program = true;

	int id = V_UBIPOSITION;
	J_POSITION = equids->find("ubiposition", id);

	//! Send an error message or also quit program..
	if (J_POSITION==-1) {
		std::cout << "Not defined jockey \"position\"" << std::endl;
		continue_program = true;
	}

	id = V_LASERSCAN;
	J_LASER_RECOGNITION = equids->find("laserscan", id);
	if (J_LASER_RECOGNITION==-1) {
		std::cout << "Not defined jockey \"laserscan\"" << std::endl;
		continue_program = false;
	}

	//equids->initJockey(J_POSITION);

	equids->initJockey(J_LASER_RECOGNITION);

	return continue_program;
}

void LaserExplorationScenario::Run() {
	CMessage msg;
	MappedObjectPosition position;
	int len = sizeof(struct MappedObjectPosition);
	msg.data = new uint8_t[len];
	msg.len = len;
	while (!quit) {
		switch(state) {
		case S_START:
			//equids->switchToJockey(J_POSITION);
//			state = S_RANDOM_EXPLORATION;
//			equids->switchToJockey(J_LASER_RECOGNITION);
			state = S_RECOGNITION;
			break;
		case S_RANDOM_EXPLORATION: {
			// run this forever for now
			CMessage msg = equids->getMessage(J_POSITION);
			if (msg.type==MSG_UBISENCE_POSITION) {
				std::cout << "Position is: " << msg.data << std::endl;
			}
			break;
		}
		case S_RECOGNITION: {
			std::cout << "Send recognition message of size " << len << std::endl;
			// send message to recognize object at this location
			position.mappedBy = -1;
			position.type = UNIDENTIFIED;
			position.map_id = -1;
			position.xPosition = -1.0;
			position.yPosition = -1.0;
			position.zPosition = -1.0;
			position.phiPosition = -1.0;
			msg.type = MSG_LASER_DETECT_STEP;
			memcpy(msg.data, &position, len);
			equids->sendMessage(J_LASER_RECOGNITION, msg);
			std::cout << "Recognition message sent" << std::endl;
			sleep(4);
			break;
		}
		case S_QUIT:
			quit = true;
			break;
		}
		usleep(100000);
	}
}



