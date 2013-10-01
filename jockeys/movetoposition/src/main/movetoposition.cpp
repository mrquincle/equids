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
	WAIT = 0, MOVING, STANDING
} ActualCalibrationState;

#define WAIT_QUEUE 10
using namespace std;
static RobotPosition* finishPosition;
static ActualCalibrationState actualTask = WAIT;
static ActualCalibrationState recvPos;
static ActualCalibrationState waitPos[WAIT_QUEUE];
static int waitTime[WAIT_QUEUE], recvTime;
static int wait_ptr;
static int delayTime;
static RobotBase::RobotType robot_type;
static RobotBase* robot;
//message system
static CMessageServer* message_server;
static CMessage messagee;
static std::string portMS;
static move_to* mv;
static RobotPosition endPosition;
static UbiPosition detectedPosition;
static UbiPosition detectedPositionOld1;
static UbiPosition detectedPositionOld2;
static UbiPosition detectedPositionOld3;
static UbiPosition detectedPositionOld4;
static int onPositionHist;

//bridles
static CMotors* motor;
static CTimer* timer;

static bool stop = false;
static bool gSet = false;
static int onPosition = 0;
static int lastOnPos = -1;
static int pos = 0;
static int goAhead;

void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
//RobotBase::MSPReset();
		exit(0);
	}
}

static void reset_jockey() {
   int i;
   pos = 0;
   recvPos = STANDING;
   for (i=0; i<WAIT_QUEUE; i++) {
      waitTime[i] = -1;
      waitPos[i] = STANDING;
   }
   recvTime = -1;
   wait_ptr = 0;
   onPositionHist=2;
   goAhead=-1;
}

void initialize() {
	robot_type = RobotBase::Initialize(NAME);
	robot = RobotBase::Instance();
   reset_jockey();   

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
	motor = new CMotors(robot, robot_type);
	std::cout << "init motors" << std::endl;
	motor->init();
	std::cout << "after motor init" << std::endl;
	motor->setSpeeds(0, 0);
	std::cout << "after motor init" << std::endl;
	usleep(20000);
}

static void collision() {
   motor->setSpeeds(0, 0);
   RobotPosition collisionPos;
   collisionPos.x = detectedPosition.x;
   collisionPos.y = detectedPosition.y;
   collisionPos.phi = (float) motor->getPosition()[2];
   message_server->sendMessage(MSG_COLLISION_DETECTED,
     &collisionPos, sizeof(RobotPosition));
   actualTask = WAIT;
   reset_jockey();
}

static void remove_wait() {
   int i;
   if (wait_ptr>1) {
      for (i=0; i<wait_ptr-1; i++) {
         waitTime[i]=waitTime[i+1];
         waitPos[i]=waitPos[i+1];
      }
   }
   wait_ptr--;
}

static void readMessages() {
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
         reset_jockey();
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
				stop = true;
			}
			break;
		}
		case MSG_MOVETOPOSITION: {
			printf("msg movetopos \n\n\n\n");

			{
				//actualTask = MOVING;

				finishPosition = new RobotPosition();
				printf("setting final position \n");
				memcpy(finishPosition, messagee.data, sizeof(RobotPosition));
				usleep(10000);
				gSet = true;
			}
			break;
		}
		case MSG_UBISENCE_POSITION: 
			if (actualTask != WAIT) {
             memcpy(&detectedPosition, messagee.data, sizeof(UbiPosition));
             recvTime = timer->getTime();
             printf("msg UBI pos stamp %i time %i pos (%f, %f)\n", detectedPosition.time_stamp, recvTime, detectedPosition.x,
                           detectedPosition.y);
             if (pos==0) { 
               pos++;
               detectedPositionOld4 = detectedPositionOld3;
               detectedPositionOld3 = detectedPositionOld2;
               detectedPositionOld2 = detectedPositionOld1;
               detectedPositionOld1 = detectedPosition;
             } else if (fabs(detectedPosition.y - detectedPositionOld1.y) + 
                fabs(detectedPosition.x - detectedPositionOld1.x) > 0.035) {
               pos++;
               detectedPositionOld4 = detectedPositionOld3;
               detectedPositionOld3 = detectedPositionOld2;
               detectedPositionOld2 = detectedPositionOld1;
               detectedPositionOld1 = detectedPosition;
               if (onPosition==0) {
                  if (onPositionHist>0) {
                     onPositionHist--;
                  }
               }
               printf("MOVING 3cm %i, onPosition %i hist %i\n", timer->getTime(), onPosition, onPositionHist);
               if (recvPos==STANDING) {
                  if (wait_ptr>0 && waitPos[0]==MOVING) {
                     delayTime = recvTime-waitTime[0];
                     printf("UBI DELAY is %f\n", delayTime/1000.0);
                     remove_wait();
                  }
               }
               if (wait_ptr>0) {
                  while ((wait_ptr>0) && recvTime-waitTime[0]>10000) {
                     remove_wait();
                  }
               }
               goAhead = -1;
               recvPos = MOVING;
             } else if (detectedPosition.time_stamp-detectedPositionOld1.time_stamp>1) {
                if (recvPos==MOVING) {
                   recvPos = STANDING;
                   goAhead = 4;
                   detectedPositionOld1 = detectedPosition;
                }
                if (goAhead==0) {
                   printf("Collision detected\n");
                   collision();
                }
             }
          }
			
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
   std::cout << "################################################################################" << std::endl;
   std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;
   std::cout << "################################################################################" << std::endl;

   
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
	int turned = 0;

	printf("goal position set\n");

	while (!stop) {
		switch (actualTask) {
		case WAIT: {
			readMessages();
			usleep(200000);
		}
			break;
		default: {

			readMessages();

         if (actualTask!=WAIT) {
         if (goAhead>0) {
            printf("Drive ahead to detect COLLISION\n");
            if (robot_type==RobotBase::ACTIVEWHEEL) {
               motor->setSpeeds(60, 0);
            } else {
               motor->setSpeeds(45, 0);
            }
            onPosition = 0;
            goAhead--;
            usleep(500000);
         } else if (onPosition == 1) {
				turned = mv->turn(*finishPosition);
				usleep(50000);
			} else {
				if (pos >= 4) {
					//printf("move\n");
					//onPosition = mv->move(*finishPosition, *detectedPosition);
					if (fabs(detectedPosition.y - detectedPositionOld4.y) > 0.04
							|| fabs(detectedPosition.x - detectedPositionOld4.x)
									> 0.04) {
						if (onPositionHist == 0) {
                     printf("act pos (%f, %f) old pos (%f, %f)\n", detectedPosition.x,
                           detectedPosition.y, detectedPositionOld4.x,
                           detectedPositionOld4.y);
							motor->setMotorPosition(detectedPosition.x,
									detectedPosition.y,
									(atan2(
											detectedPosition.y
													- detectedPositionOld4.y,
											detectedPosition.x
													- detectedPositionOld4.x)));

						}

						onPosition = mv->move(*finishPosition,
								detectedPosition);
						//motor->setSpeeds(motor->calibratedSpeed, 0);
						//usleep(500000);
						//motor->setSpeeds(0,0);
					}

				} else {
					printf("Drive ahead for HEADING\n");
					motor->setSpeeds(60, 0);
					onPosition = 0;
				}
				usleep(500000);
			}
         if ((onPosition==0)!=(lastOnPos==0)) {
            if (onPosition>0) {
               waitPos[wait_ptr]=STANDING;
               printf("CHANGE TO STANDING in time %i\n", waitTime[wait_ptr]);
            } else {               
               waitPos[wait_ptr]=MOVING;
               printf("CHANGE TO MOVING in time %i\n", waitTime[wait_ptr]);
            }
            if (wait_ptr==0) {
               wait_ptr++;
            }
            lastOnPos = onPosition;
         }
         if (onPosition>0) {
            onPositionHist=4;
         }
		
			if (turned == 1) {
				printf("turned == 1\n");
				motor->setSpeeds(0, 0);
				actualTask = WAIT;
				endPosition.x = detectedPosition.x;
				endPosition.y = detectedPosition.y;
				endPosition.phi = (float) motor->getPosition()[2];
				printf("sending MSG_MOVETOPOSITION_DONE\n");
				message_server->sendMessage(MSG_MOVETOPOSITION_DONE,
						&endPosition, sizeof(endPosition)); //todo &

			}
		}
      }
			break;
		}

	}

	return 0;
}

