/**
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * @author    Robert Penicka
 * @date      Aug 14, 2013
 * @project   Replicator
 * @company   CVUT
 * @company   Gerstner laboratory
 * @case      getting position
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

#include <wapi/wapi.h>
#include <CUbisencePosition.h>
#include <CMessageServer.h>
#include <CMessage.h>

#define __DEBUG__
#define UBISENCE_CHANNEL 55
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

//! The name of the controller can be used for controller selection
std::string NAME = "UbiPosition";
CUbisencePosition* ubisencePositionServer;
typedef enum {
	WAIT = 0, SEND_POSITION
} ActualUbisencePositionState;
ActualUbisencePositionState actualTask = WAIT;
CMessageServer* message_server;
CMessage messagee;
int error = 0;
std::string portMS;
bool stop;

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		ubisencePositionServer->stopServer();
		//RobotBase::MSPReset();
		stop = true;
	}
}

//using namespace wapi;

void readMessages() {
	messagee = message_server->getMessage();
	if (messagee.type != MSG_NONE) {
		//	fprintf(stdout,"Command: %s %i %i %i %i\n",message.getStrType(),message.value1,message.value2,message.value3,message.value4);
		switch (messagee.type) {
		case MSG_INIT: {
			printf("message init\n");
			ubisencePositionServer = new CUbisencePosition();
			actualTask = WAIT;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_START: {
			actualTask = SEND_POSITION;
			if(ubisencePositionServer->stop){
			ubisencePositionServer->initServer(UBISENCE_CHANNEL);
			}
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_STOP: {
			actualTask = WAIT;
			printf("stopping ubisence position server\n");
			//ubisencePositionServer->stopServer();
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_QUIT: {
			ubisencePositionServer->stopServer();
			actualTask = WAIT;
			stop = true;
		}
			break;
		default:
			break;
		}
	}
}

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {

	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	if (argc > 1) {
		portMS = std::string(argv[1]);
	} else {
		std::cout << DEBUG << "Usage: message_server_port_number" << std::endl;
		return 1;
	}

	std::cout << "Create (receiving) message server on port " << portMS
			<< std::endl;

	message_server = new CMessageServer();
	std::cout << "Initialize CMessageServer" << std::endl;
	message_server->initServer(portMS.c_str());

	while (!stop) {
		readMessages();
		switch (actualTask) {
		case SEND_POSITION: {
			if (ubisencePositionServer->validPosition) {

				UbiPosition posit = ubisencePositionServer->getPosition();
#ifdef __DEBUG__
					printf("position is %f %f %f %d \n",posit.x,posit.y,posit.z,posit.time_stamp);
#endif
				message_server->sendMessage(MSG_UBISENCE_POSITION, &posit,
						sizeof(UbiPosition));
			}else{
				error +=1;
			}
		}
			break;
		case WAIT: {
			usleep(100000);
		}
			break;
		default:
			break;
		}
		usleep(250000);
	}

	if (ubisencePositionServer != NULL) {
		delete ubisencePositionServer;
	}
	std::cout << "Finish UbiPosition" << std::endl;
	return 0;
}

