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

#include "../eth/CEquids.h"
#include "../eth/CMessage.h"

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "ActionSelection";

typedef enum
{
   S_FORTH = 0,
   S_BACK,
   S_QUIT
} TState;


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



    int forth = equids.find("forth");
    int back = equids.find("back");

    if (forth==-1) {
       std::cout << "Not deffined jockey forth" << std::endl;
       equids.quit();
       return EXIT_FAILURE;
    }
    if (back==-1) {
       std::cout << "Not deffined jockey back" << std::endl;
       equids.quit();
       return EXIT_FAILURE;
    }
    
    TState state = S_FORTH;
    int num=0;
    bool quit = false;

    equids.initJockey(forth);
    while (!quit) {
       switch(state) {
          case S_FORTH:
             sleep(1);
             equids.switchToJockey(back);
             state = S_BACK;
             break;
          case S_BACK:
             sleep(1);
             num++;
             if (num>3) {
                state = S_QUIT;
                std::cout << "Quit system" <<std::endl;
             } else {
                equids.switchToJockey(forth);
                state = S_FORTH;
             }
             break;
          case S_QUIT:
             quit = true;
             break;
       }
       usleep(50000);
    }
             
	equids.quit();
	return 0;
}


