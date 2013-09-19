/*
 *  Created on:14. 8. 2013
 *      Author: Vojtech
 */
#include <sys/types.h>
#include <IRobot.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "termios.h"
#include <CMessageServer.h>
#include <CTimer.h>
#include <signal.h>
#include "../eth/messageDataType.h"
#include <CMotors.h>
#include "../move_to/moveto.h"

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

#define DEBUGODOCALIB
#define NAME "Moving"
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "
#define UBISENCE_POSITION_CHANNEL 55
#define UBISENCE_MESSAGE_SERVER_CHANNEL 56
#define DEBUGSTRING NAME << '[' << getpid() << "] " << __func__ << "(): "
typedef enum {
	WAIT = 0, MOVING
} ActualCalibrationState;

using namespace std;
RobotPosition* finishPosition;
ActualCalibrationState actualTask = WAIT;
RobotBase::RobotType robot_type;
RobotBase* robot;
//message system
CMessageServer* message_server;
CMessage messagee;
std::string portMS;
move_to* mv;
RobotPosition endPosition;
UbiPosition detectedPosition;
UbiPosition detectedPositionOld3;
UbiPosition detectedPositionOld1;
UbiPosition detectedPositionOld2;

//bridles
CMotors* motor;
CTimer* timer;

//actions hadlers

bool stop = false;
bool gSet = false;
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
//RobotBase::MSPReset();
		exit(0);
	}
}
int pos = 0;

void initialize() {
	robot_type = RobotBase::Initialize(NAME);
	robot = RobotBase::Instance();

	for (int i = 0; i < 4; ++i)
		robot->SetPrintEnabled(i, false);
	switch (robot_type) {
	case RobotBase::UNKNOWN:
		std::cout << "Detected unknown robot" << std::endl;
		break;
	case RobotBase::KABOT:
		std::cout << "Detected Karlsruhe robot" << std::endl;
		break;
	case RobotBase::ACTIVEWHEEL: {
		std::cout << "Detected Active Wheel robot" << std::endl;
	}
		break;
	case RobotBase::SCOUTBOT:
		std::cout << "Detected Scout robot" << std::endl;
		break;
	default:
		std::cout << DEBUG
				<< "No known type (even not unknown). Did initialization go well?"
				<< std::endl;
	}

	std::cout << "Timer inicialization" << std::endl;
	timer = new CTimer();
	timer->start();

	std::cout << "Create motor object" << std::endl;
	motor = new CMotors(robot, robot_type, timer);
	std::cout << "init motors" << std::endl;
	motor->init();
	std::cout << "after motor init" << std::endl;
	motor->setSpeeds(0, 0);
	std::cout << "after motor init" << std::endl;
	usleep(20000);
}

void readMessages() {
	messagee = message_server->getMessage();
	//fprintf(stdout, "Message type: %s \n", messagee.getStrType());

	if (messagee.type != MSG_NONE) {
		switch (messagee.type) {
		case MSG_INIT: {
			printf("msg init \n");
			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}
			initialize();
			std::cout << "after initialize()" << std::endl;
			mv = new move_to(motor, robot_type);

			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);

		}
			;
			break;
		case MSG_START: {
			printf("msg start \n");

			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}
			actualTask = MOVING;
			if (robot_type == RobotBase::ACTIVEWHEEL) {
				ActiveWheel *bot = (ActiveWheel*) robot;
				printf("changing hinge \n");
				bot->MoveHingeToAngle(8.3);
				//motor->getPosition()[5] = 2.996730326; //setting hinge to 171.7°
				usleep(500000);
			}

			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_STOP: {
			motor->setSpeeds(0, 0);
			usleep(5000);
			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			actualTask = WAIT;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_QUIT: {
			{
				motor->setSpeeds(0, 0);
				motor->setSpeeds(0, 0);
				usleep(10000);
				robot->pauseSPI(true);
				actualTask = WAIT;
				//stop = true;
			}
			break;
		}
		case MSG_MOVETOPOSITION: {
			printf("msg movetopos \n");

			{
				//actualTask = MOVING;
				if (finishPosition == NULL) {
					finishPosition = new RobotPosition();
				}
				memcpy(finishPosition, messagee.data, sizeof(RobotPosition));
				usleep(10000);
				gSet = true;
			}
			break;
		}
		case MSG_UBISENCE_POSITION: {
			printf("msg ubisense \n");

			pos++;
			detectedPositionOld3 = detectedPositionOld2;
			detectedPositionOld2 = detectedPositionOld1;
			detectedPositionOld1 = detectedPosition;
			memcpy(&detectedPosition, messagee.data, sizeof(UbiPosition));
			printf("detected position %f %f \n", detectedPosition.x,
					detectedPosition.y);

		}
			;
			break;
		default:
			break;
		}
	}
}

/**
 * Initializes
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
	int onPosition = 0;
	int turned = 0;

	while (!gSet) {
		readMessages();
		usleep(5000);
	}
	printf("goal position set\n");

	while (!stop) {
		switch (actualTask) {
		case (WAIT): {
			readMessages();
			usleep(200000);
		}
			break;
		default: {

			readMessages();

			if (onPosition == 1) {
				turned = mv->turn(*finishPosition);
				usleep(50000);
			} else {
				if (pos > 4) {
					//printf("move\n");
					//onPosition = mv->move(*finishPosition, *detectedPosition);
					if (fabs(detectedPosition.y - detectedPositionOld3.y) > 0.03
							|| fabs(detectedPosition.x - detectedPositionOld3.x)
									> 0.03) {
						if (onPosition == 0) {
							motor->setMotorPosition(detectedPosition.x,
									detectedPosition.y,
									(atan2(
											detectedPosition.y
													- detectedPositionOld3.y,
											detectedPosition.x
													- detectedPositionOld3.x)));

						}

						onPosition = mv->move(*finishPosition,
								detectedPosition);
						//motor->setSpeeds(motor->calibratedSpeed, 0);
						//usleep(500000);
						//motor->setSpeeds(0,0);
					} else {

						if (detectedPosition.x == detectedPositionOld3.x
								&& detectedPosition.y == detectedPositionOld3.y
								&& detectedPosition.time_stamp
										!= detectedPositionOld3.time_stamp) {
							printf("kolize\n");
							motor->setSpeeds(-40, 0);
							sleep(2);
							motor->setSpeeds(0, 0);
							usleep(5000);
							message_server->sendMessage(MSG_MOVETOPOSITION_DONE,
									&endPosition, sizeof(endPosition));
							actualTask = WAIT;
						} else {
							printf("neni dostatecne velka zmena pozice\n");
							motor->setSpeeds(40, 0);
						}
					}
				} else {
					printf("jedu rovne pro zjisteni HEADING\n");
					motor->setSpeeds(40, 0);

				}
				usleep(500000);
			}

			if (turned == 1) {
				printf("turned == 1\n");
				motor->setSpeeds(0,0);
				actualTask = WAIT;
				endPosition.x = detectedPosition.x;
				endPosition.y = detectedPosition.y;
				endPosition.phi = (float) motor->getPosition()[2];
				message_server->sendMessage(MSG_MOVETOPOSITION_DONE,
						&endPosition, sizeof(endPosition)); //todo &

			}
		}
			break;
		}

	}

	return 0;
}

