/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file BackandforthScenario.cpp
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
 * @case      Sensor fusion
 */

#include <BackandforthScenario.h>
#include <iostream>

#include <global.hh>
#include <CMessage.h>

BackandforthScenario::BackandforthScenario(CEquids * equids): CScenario(equids) {
	J_BACK = J_FORTH = J_WENGUO = -1;

	state = S_FORTH;
	cnt = 0;
	num = 0;

	quit = false;
}

BackandforthScenario::~BackandforthScenario() {

}

bool BackandforthScenario::Init() {
	bool continue_program = true;

	int id = V_BACKANDFORTH;
	J_FORTH = equids->find("forth", id);
	id = V_BACKANDFORTH;
	J_BACK = equids->find("back", id);
	id = V_WENGUO;
	J_WENGUO = equids->find("wenguo", id);

	//! Send an error message or also quit program..
	if (J_FORTH==-1) {
		std::cout << "Not defined jockey forth" << std::endl;
		continue_program = false;
	}
	if (J_BACK==-1) {
		std::cout << "Not defined jockey back" << std::endl;
		continue_program = false;
	}
	if (J_WENGUO==-1) {
		std::cout << "Not defined jockey wenguo" << std::endl;
		continue_program = true;
	}

	equids->initJockey(J_FORTH);

	return continue_program;
}

void BackandforthScenario::Run() {
	while (!quit) {
		switch(state) {
		case S_FORTH:
			sleep(1);
			equids->switchToJockey(J_BACK);
			state = S_BACK;
			break;
		case S_BACK:
			sleep(1);
			num++;
			if (num>2) {
				if (J_WENGUO==-1) {
					state = S_QUIT;
					std::cout << "Quit system" <<std::endl;
				} else {
					uint8_t cmd_data[3];
					cmd_data[0] = 2; //recruiting side: 0 -- front, 1 -- left, 2 -- back, 3 -- right
					cmd_data[1] = 3; //recruited robot type: 1 -- KIT, 2 -- Scout, 3 -- ActiveWheel
					cmd_data[2] = 0; //recruited robot side: 0 -- front, 1 -- left, 2 -- back, 3 -- right
					equids->sendMessage(J_WENGUO, DAEMON_MSG_RECRUITING, cmd_data, sizeof(cmd_data));
					equids->switchToJockey(J_WENGUO);
					state = S_RECRUITING;
					equids->sendMessage(J_WENGUO, DAEMON_MSG_STATE_REQ, NULL, 0);
					cnt = 0;
				}
			} else {
				equids->switchToJockey(J_FORTH);
				state = S_FORTH;
			}
			break;
		case S_RECRUITING:
			uint8_t st;
			if (cnt<60) { // one minut wait for recruiting
				CMessage mess=equids->getMessage(J_WENGUO);
				if (mess.type==DAEMON_MSG_STATE) {
					st = mess.data[0];
					printf("J_WENGUO state is %i\n", st);
					if (st == INORGANISM) {
						printf("Quit Goal finished\n");
						state = S_QUIT;
					} else {
						sleep(1);
						cnt++;
						mess.type==0;
						equids->sendMessage(J_WENGUO, DAEMON_MSG_STATE_REQ, NULL, 0);
					}
					mess.type==0;
				}
			} else {
				printf("Quit TIME elapsed\n");
				state = S_QUIT;
			}
			break;
		case S_QUIT:
			quit = true;
			break;
		}
		usleep(100000);
	}

}
