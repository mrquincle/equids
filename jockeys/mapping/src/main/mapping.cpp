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
#include <messageDataType.h>
#include <CMotors.h>
#include <Map.h>
#include <Mapping.h>

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

#define DEBUGODOCALIB
#define NAME "Mapping"
#define MINIMAL_MAP_SIZE 4
#define WAIT_FOR_NO_MOVING 3
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "
#define UBISENCE_POSITION_CHANNEL 55
#define UBISENCE_MESSAGE_SERVER_CHANNEL 56
#define DEBUGSTRING NAME << '[' << getpid() << "] " << __func__ << "(): "
typedef enum {
	WAIT = 0, MAPPING
} ActualCalibrationState;

using namespace std;

ActualCalibrationState actualTask = WAIT;
RobotBase::RobotType robot_type;
RobotBase* robot;
//message system
CMessageServer* message_server;
CMessage messagee;
std::string portMS;

//bridles
CMotors* motor;
CTimer* timer;
int wait_no_moving = WAIT_FOR_NO_MOVING;

//actions hadlers
Map* slamMap = NULL;
Mapping* mapProcedure = NULL;
bool stop = false;
DetectedBlob* detectedBlob = NULL;
UbiPosition* ubiposition = NULL;
UbiPosition* initialUbiPosition = NULL;
UbiPosition* endingUbiPosition = NULL;

void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
//RobotBase::MSPReset();
		exit(0);
	}
}

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
bool isPossible(DetectedBlob* detectedBlob) {
	return (detectedBlob->x < 8 && detectedBlob->x > -8 && detectedBlob->y < 8
			&& detectedBlob->y > -8 && detectedBlob->z < 4
			&& detectedBlob->z > -4);
}

void readMessages() {

	messagee = message_server->getMessage();

	if (messagee.type != MSG_NONE) {
		switch (messagee.type) {
		case MSG_INIT: {
			initialize();
			std::cout << "after initialize()" << std::endl;
			actualTask = WAIT;
			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);

		}
			;
			break;
		case MSG_START: {

			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}
			actualTask = MAPPING;
			if (robot_type == RobotBase::ACTIVEWHEEL) {
				ActiveWheel *bot = (ActiveWheel*) robot;
				printf("changing hinge \n");
				bot->MoveHingeToAngle(8.3);
				motor->getPosition()[5] = 2.996730326; //setting hinge to 171.7°
				usleep(500000);

			}

			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_STOP: {
			motor->setSpeeds(0, 0);
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
				stop = true;
			}
			break;
		}
		case MSG_CAM_DETECTED_BLOB: {
			//	printf("MSG_CAM_DETECTED_BLOB motor calibration %d\n",message_server->messageRead);

			if (messagee.len != 0) {
				DetectedBlobWSize* detected = new DetectedBlobWSize();
				memcpy(detected, messagee.data, sizeof(DetectedBlobWSize));
				if (detectedBlob != NULL) {
					delete detectedBlob;
					detectedBlob = NULL;
				}
				detectedBlob = new DetectedBlob();
				detectedBlob->x = detected->detectedBlob.x;
				detectedBlob->y = detected->detectedBlob.y;
				detectedBlob->z = detected->detectedBlob.z;
				detectedBlob->phi = detected->detectedBlob.phi;
				printf("detected blob MAPPING %f %f %f %f \n",
						detected->detectedBlob.x, detected->detectedBlob.y,
						detected->detectedBlob.z, detected->detectedBlob.phi);
			}
		}
			;
			break;
		case MSG_MOTOR_CALIBRATION_RESULT: {
			//	printf("calibrating mapping motor......\n");
			MotorCalibResult* calibrationResult = new MotorCalibResult();
			memcpy(calibrationResult, messagee.data, sizeof(MotorCalibResult));
			motor->calibrate(*calibrationResult);
			delete calibrationResult;
		}
			;
			break;
		case MSG_UBISENCE_POSITION: {
			//	printf("Ubisence position message ......\n");

			if (ubiposition == NULL) {
				ubiposition = new UbiPosition();
			}
			//	printf("after ubiposition receive\n");
			memcpy(ubiposition, messagee.data, sizeof(UbiPosition));
			//	printf(" %f %f %f %d \n", ubiposition->x, ubiposition->y,ubiposition->z, ubiposition->time_stamp);

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
	std::cout << "Initialized CMessageServer" << std::endl;
	while (!stop) {
		if (detectedBlob != NULL) {
			delete detectedBlob;
			detectedBlob = NULL;
		}

		readMessages();
		switch (actualTask) {
		case MAPPING: {
			// detect only when motor is stopped
			if (ubiposition == NULL) {
				//wait until usbisence postion is present to initialize position of motors
				break;
			} else {
				if (initialUbiPosition == NULL) {
					//initialize motor position
					printf("inititial position: %f %f ..........",
							ubiposition->x, ubiposition->y);
					motor->setMotorPosition(ubiposition->x, ubiposition->y, 0);
					printf("after motor: %f %f ..........",
							motor->getPosition()[0], motor->getPosition()[1]);
					mapProcedure = new Mapping(motor);
					slamMap = new Map(motor->getPosition(), robot_type);
					initialUbiPosition = new UbiPosition();
					memcpy(initialUbiPosition, ubiposition,
							sizeof(UbiPosition));

				}

				if (endingUbiPosition == NULL) {
					//drive forward until difference in ubiposition is enough
					float actualposition[2] = { ubiposition->x, ubiposition->y };
					float initialposition[2] = { initialUbiPosition->x,
							initialUbiPosition->y };

					if (euclideanDistancef(actualposition, initialposition)
							> 0.3) {
						//wait for true position
						for (int var = 0; var < 20; ++var) {
							usleep(100000);
							readMessages();
						}
						endingUbiPosition = new UbiPosition();
						memcpy(endingUbiPosition, ubiposition,
								sizeof(UbiPosition));
						double ubisenceangle = atan2(
								initialUbiPosition->y - endingUbiPosition->y,
								initialUbiPosition->x - endingUbiPosition->x);
						motor->setMotorPosition(endingUbiPosition->x,
								endingUbiPosition->y, ubisenceangle);
					} else {
						motor->setSpeeds(motor->calibratedSpeed, 0);
					}
				} else {

					bool seeBlob = false;
					//printf("position: x_%f y_%f phi_%f\n",motor->getPosition()[0],motor->getPosition()[1],motor->getPosition()[2]*180/3.14159265);
					if (!motor->isMoving()
							&& mapProcedure->wait_stopped
									< mapProcedure->image_wait_count - 5) {

						if (detectedBlob != NULL
								&& (isPossible(detectedBlob))) {
							double measuredCirclePos[3] = { detectedBlob->x,
									detectedBlob->y, detectedBlob->phi };
							slamMap->filter(motor->getPosition(),
									measuredCirclePos);
							seeBlob = true;
						} else {
							slamMap->odometryChange(motor->getPosition());
						}
					} else {
						slamMap->odometryChange(motor->getPosition());
					}

					//test whether map is enough sized
					if (slamMap->velikostmapy < MINIMAL_MAP_SIZE
							|| mapProcedure->wait_stopped > 0
									&& mapProcedure->actualState
											!= Mapping::DRIVE_FORWARD) {
						if (slamMap->newDetected) {
							slamMap->newDetected = false;
							mapProcedure->wait_stopped += 100;
						}
						if (slamMap->seeAfterLongTime) {
							slamMap->seeAfterLongTime = false;
							mapProcedure->wait_stopped += 50;
						}

						mapProcedure->doMappingMotion(seeBlob);
						printf("%d\n", mapProcedure->wait_stopped);
					} else {

						//send map
						actualTask = WAIT;
						message_server->sendMessage(MSG_MAP_DATA,
								slamMap->state->data,
								sizeof(double)
										* (slamMap->state->size1
												* slamMap->state->size2));
						while (message_server->getMessage().type
								!= MSG_ACKNOWLEDGE) {
							usleep(10000);
						}
						message_server->sendMessage(MSG_MAP_COVARIANCE,
								slamMap->P->data,
								sizeof(double)
										* (slamMap->P->size1 * slamMap->P->size2));
						while (message_server->getMessage().type
								!= MSG_ACKNOWLEDGE) {
							usleep(10000);
						}
						std::cout << " Map sended " << std::endl;
					}
				}
			}
			usleep(50000);
		}
			break;
		case WAIT: {
			usleep(100000);
		}
			break;
		default:
			usleep(100000);
			break;
		}
		usleep(100000);
	}

	motor->setMotorPosition(0, 0, 0);

	motor->setSpeeds(0, 0);

	delete motor;
	delete ubiposition;
	delete slamMap;
	delete mapProcedure;
	delete message_server;
	std::cout << "Exit Motor Calibration" << std::endl;
	return 0;
}
