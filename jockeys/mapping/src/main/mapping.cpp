#include <sys/types.h>
#include <IRobot.h>
#include <comm/Ethernet.h>
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
#include <CLeds.h>
#include <wapi/wapi.h>
#include <iostream>
#include <iomanip>

using namespace wapi;
using namespace std;

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

#define DEBUGODOCALIB
#define NAME "Mapping"
#define MINIMAL_RUNS 1
#define WAIT_FOR_NO_MOVING 2
#define DEBUGSTRING NAME << '[' << getpid() << "] " << __func__ << "(): "
#define UBISENCE_POSITION_CHANNEL 55
#define UBISENCE_MESSAGE_SERVER_CHANNEL 56
typedef enum {
	WAIT = 0, MAPPING
} ActualMappingState;

using namespace std;
using namespace Ethernet;

ActualMappingState actualTask = WAIT;
RobotBase::RobotType robot_type;
RobotBase* robot;
CLeds* leds;
//message system
CMessageServer* message_server;
CMessage messagee;
std::string portMS;
int myID = -1;

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
		std::cout << DEBUGSTRING << "Detected unknown robot" << std::endl;
		break;
	case RobotBase::KABOT:
		std::cout << DEBUGSTRING << "Detected Karlsruhe robot" << std::endl;
		break;
	case RobotBase::ACTIVEWHEEL: {
		std::cout << DEBUGSTRING << "Detected Active Wheel robot" << std::endl;
	}
		break;
	case RobotBase::SCOUTBOT:
		std::cout << DEBUGSTRING << "Detected Scout robot" << std::endl;
		break;
	default:
		std::cout << DEBUGSTRING
				<< "No known type (even not unknown). Did initialization go well?"
				<< std::endl;
	}

	char* robotID = getenv("sr_id");
	if (robotID != NULL) {
		myID = atoi(robotID);
	}
	std::cout << DEBUGSTRING << "Robot id is: " << myID << std::endl;

	std::cout << DEBUGSTRING << "Timer initialization" << std::endl;
	timer = new CTimer();
	timer->start();

	std::cout << DEBUGSTRING << "Create motor object" << std::endl;
	motor = new CMotors(robot, robot_type);
	std::ostringstream msg; msg.clear(); msg << NAME << '[' << getpid() << "] ";
	motor->setLogPrefix(msg.str());
	std::cout << DEBUGSTRING << "Create CLeds object" << std::endl;
	leds = new CLeds(robot,robot_type);
	msg.clear(); msg << NAME << '[' << getpid() << "] ";
	leds->setLogPrefix(msg.str());
	std::cout << DEBUGSTRING << "init motors" << std::endl;
	motor->init();
	std::cout << DEBUGSTRING << "after motor init" << std::endl;
	motor->setSpeeds(0, 0);
	std::cout << DEBUGSTRING << "after motor init" << std::endl;
	usleep(20000);
	slamMap = new Map(motor->getPosition(), robot_type,myID);
}
bool isPossible(DetectedBlob* detectedBlob) {
	return (detectedBlob->x < 8 && detectedBlob->x > -8 && detectedBlob->y < 8
			&& detectedBlob->y > -8 && detectedBlob->z < 4
			&& detectedBlob->z > -4);
}

void sendMap(){
	if(slamMap!=NULL){
	for (int var = 0; var < slamMap->mapSize; ++var) {
		MappedObjectPosition pos = slamMap->getMappedPosition(var);
		pos.mappedBy = myID;

		CMessage packedMessage;
		uint64_t broadcast = Ubitag::BROADCAST;
		printf("packing object num %d \n",var);
		packedMessage = CMessage::packToZBMessage(
				broadcast, MSG_MAP_DATA,
				&pos,
				sizeof(MappedObjectPosition));
		printf("sending object num %d \n",var);

		message_server->sendMessage(MSG_ZIGBEE_MSG,
				packedMessage.data, packedMessage.len);
		usleep(200000);
	}
	}
}

void readMessages() {

	messagee = message_server->getMessage();

	if (messagee.type != MSG_NONE) {
		switch (messagee.type) {
		case MSG_INIT: {
			std::cout << DEBUGSTRING << "message init" << std::endl;
			initialize();
			std::cout << DEBUGSTRING << "after initialize()" << std::endl;
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
			//	actualTask = MAPPING;

			if (robot_type == RobotBase::ACTIVEWHEEL) {
				ActiveWheel *bot = (ActiveWheel*) robot;
				printf("changing hinge \n");
				bot->MoveHingeToAngle(8.3);
				motor->getPosition()[5] = 163.4/180.0*M_PI; //setting hinge to 160°
				usleep(500000);

			}
			leds->color(LC_OFF);
			leds->color(LC_ORANGE);
			actualTask = MAPPING;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_STOP: {
			motor->setSpeeds(0, 0);
			leds->color(LC_RED);
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
				/*
				 printf("detected blob MAPPING x:%f y:%f z:%f phi:%f \n",
				 detected->detectedBlob.x, detected->detectedBlob.y,
				 detected->detectedBlob.z, detected->detectedBlob.phi);*/
				/*
				 float pok[4]={detected->detectedBlob.x, detected->detectedBlob.y, detected->detectedBlob.phi,
				 detected->detectedBlob.z};
				 Map::convertCameraMeasurement(pok,motor->getPosition()[5],robot_type);
				 printf("converted MAPPING x:%f y:%f z:%f phi:%f \n",pok[0],pok[1],pok[3],pok[2]);
				*/
			}
		}
			;
			break;
		case MSG_MOTOR_CALIBRATION_RESULT: {
				printf("calibrating mapping motor......\n");
			MotorCalibResult calib;
						memcpy(&calib,messagee.data,messagee.len);
						printf("calibration succesfully readed: %e %e %e %d\n",
								calib.odometry_koef1, calib.odometry_koef2,
								calib.odometry_koef3,calib.calibratedSpeed);
			motor->calibrate(calib);

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
			//	printf("robot position: %f %f %f %d \n", ubiposition->x, ubiposition->y,ubiposition->z, ubiposition->time_stamp);

		}
			;
			break;
		case MSG_MAP_DATA: {
			MappedObjectPosition mapedObject;
			memcpy(&mapedObject, messagee.data, sizeof(MappedObjectPosition));
			printf("received object type %d pos %f %f %f %f id %d robid %d\n",mapedObject.type,mapedObject.xPosition,
					mapedObject.yPosition,	mapedObject.phiPosition,mapedObject.zPosition,mapedObject.map_id,mapedObject.mappedBy);
			printf("received map data from %d of type %d\n",mapedObject.mappedBy,mapedObject.type);
			if(mapedObject.mappedBy==myID){
				printf("save object to map\n");
				slamMap->saveObjectToMap(mapedObject);
				sendMap();

			}else{
				printf("add other objects\n");
				slamMap->addOtherRobotsObjects(mapedObject);
			}
		}
			;
			break;
		case MSG_GET_ALL_MAPPED_OBJS: {
				for (int var = 0; var < slamMap->mapSize; ++var) {
					MappedObjectPosition mapedObject = slamMap->getMappedPosition(var);
					message_server->sendMessage(MSG_MAP_DATA,&mapedObject,sizeof(MappedObjectPosition));
				}
				}
					;
					break;
		case MSG_GET_NEAREST_MAPPED_OBJECT_OF_TYPE_TO_POS: {
					NearestObjectOfTypeToThisPosition nearestTo;
					memcpy(&nearestTo, messagee.data, sizeof(NearestObjectOfTypeToThisPosition));
					printf("want to know nearest object to %f %f of type %d \n",nearestTo.xPosition,nearestTo.yPosition,nearestTo.type);
					int num =slamMap->nearestTypeID(nearestTo);
					if(num != -1){
						MappedObjectPosition mapedObject =  slamMap->getMappedPosition(num);
						message_server->sendMessage(MSG_MAP_DATA,&mapedObject,sizeof(MappedObjectPosition));
					}
				}
					;
					break;
		default:
			break;
		}
	}

}

void doColisionMotion(){
leds->color(LC_RED);
motor->setSpeeds(0,0);
usleep(50000);
double positionBesf[]={motor->getPosition()[0],motor->getPosition()[1],motor->getPosition()[2]};
double actualPos[]={motor->getPosition()[0],motor->getPosition()[1],motor->getPosition()[2]};
//drive back
printf("driving back \n");
while(euclideanDistance(positionBesf,actualPos)<0.3){
	readMessages();
	motor->setSpeeds(-motor->calibratedSpeed,0);
	usleep(50000);
	actualPos[0]=motor->getPosition()[0];
	actualPos[1]=motor->getPosition()[1];
	actualPos[2]=motor->getPosition()[2];
}
motor->setSpeeds(0,0);
usleep(50000);
//turn around
printf("turn around \n");
while(fabs(normalizeAngle(actualPos[2])-normalizeAngle(positionBesf[2]))<2.5 ){
readMessages();
motor->setSpeeds(0,motor->calibratedSpeed);
usleep(50000);
actualPos[0]=motor->getPosition()[0];
actualPos[1]=motor->getPosition()[1];
actualPos[2]=motor->getPosition()[2];
}
motor->setSpeeds(0,0);
usleep(50000);
printf("collision procedure ended \n");
}

/**
 * Initializes 
 */
int main(int argc, char **argv) {
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	std::cout << "################################################################################" << std::endl;
	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;
	std::cout << "################################################################################" << std::endl;

	if (argc > 1) {
		portMS = std::string(argv[1]);
	} else {
		std::cout << DEBUGSTRING << "Usage: message_server_port_number" << std::endl;
		return 1;
	}

	std::cout << DEBUGSTRING << "Create (receiving) message server on port " << portMS
			<< std::endl;

	message_server = new CMessageServer();
	std::cout << DEBUGSTRING << "Initialize CMessageServer" << std::endl;
	message_server->initServer(portMS.c_str());
	std::cout << DEBUGSTRING << "Initialized CMessageServer" << std::endl;

	std::ostringstream ss; ss.clear(); ss.str("");
	ss << DEBUGSTRING;
	std::string debug_str = ss.str();

	while (!stop) {
		if (detectedBlob != NULL) {
			delete detectedBlob;
			detectedBlob = NULL;
		}

		readMessages();
		switch (actualTask) {
		case MAPPING: {
			// detect only when motor is stopped
			leds->colorToggle(LC_ORANGE);
			if(robot_type == RobotBase::SCOUTBOT){
						message_server->sendMessage(MSG_MAP_COMPLETE, NULL, 0);
			}else{
			if (ubiposition == NULL) {
				//wait until usbisence postion is present to initialize position of motors
				break;
			} else {
				if (initialUbiPosition == NULL) {
					//initialize motor position
					printf("%sInitial position: %f %f ..........\n", debug_str.c_str(),
							ubiposition->x, ubiposition->y);
					motor->setMotorPosition(ubiposition->x, ubiposition->y, 0);
					printf("%sAfter motor: %f %f ..........\n", debug_str.c_str(),
							motor->getPosition()[0], motor->getPosition()[1]);
					if(mapProcedure == NULL){
					mapProcedure = new Mapping(motor);
					}

					if(slamMap==NULL){
					slamMap = new Map(motor->getPosition(), robot_type,myID);
					}
					usleep(10000);

					if(initialUbiPosition == NULL){
					initialUbiPosition = new UbiPosition();
					}
					memcpy(initialUbiPosition, ubiposition,
							sizeof(UbiPosition));

				}

				if (endingUbiPosition == NULL) {
					//drive forward until difference in ubiposition is enough
					float actualposition[2] = { ubiposition->x, ubiposition->y };
					float initialposition[2] = { initialUbiPosition->x,
							initialUbiPosition->y };
					float drivenDist=euclideanDistancef(actualposition, initialposition);
					if (drivenDist > 0.4) {
						motor->setSpeeds(0, 0);
						//wait for true position
						for (int var = 0; var < 50; ++var) {
							usleep(100000);
							readMessages();
						}
						endingUbiPosition = new UbiPosition();
						memcpy(endingUbiPosition, ubiposition,
								sizeof(UbiPosition));
						double ubisenceangle = atan2(
								endingUbiPosition->y - initialUbiPosition->y,
								endingUbiPosition->x - initialUbiPosition->x);
						switch (robot_type) {
							case RobotBase::ACTIVEWHEEL:{
								float distance_to_tag = 0.07;
								endingUbiPosition->x=endingUbiPosition->x+sin(ubisenceangle)*distance_to_tag;
								endingUbiPosition->y=endingUbiPosition->y-cos(ubisenceangle)*distance_to_tag;
							};break;
							default:{
								float distance_to_tag = 0.05;
								endingUbiPosition->x=endingUbiPosition->x+cos(ubisenceangle)*distance_to_tag;
								endingUbiPosition->y=endingUbiPosition->y+sin(ubisenceangle)*distance_to_tag;
							};break;
						}

						motor->setMotorPosition(endingUbiPosition->x,
								endingUbiPosition->y, ubisenceangle);
						printf("%sinitialUbiPosition : %f %f ..........\n", debug_str.c_str(),
								initialUbiPosition->x, initialUbiPosition->y);
						printf("%sendingUbiPosition : %f %f ..........\n", debug_str.c_str(),
								endingUbiPosition->x, endingUbiPosition->y);
						printf("%suhel : %f  ..........\n", debug_str.c_str(),ubisenceangle);

					} else {
						motor->setSpeeds(motor->calibratedSpeed, 0);
						double motorInitial[]={initialUbiPosition->x,initialUbiPosition->y};
						printf("%smotorintial %f %f \n",debug_str.c_str(),motorInitial[0],motorInitial[1]);
						double motorDriven = euclideanDistance(motor->getPosition(),motorInitial);
						std::cout << DEBUGSTRING << "motorDriven " << motorDriven << std::endl;
						printf("%sdrivenDist %f \n",debug_str.c_str(),drivenDist);
						if(drivenDist+0.4 < motorDriven){
							motor->setSpeeds(0 , 0);
							for (int var = 0; var < 30; ++var) {
								usleep(100000);
								readMessages();
							}
							actualposition[0] = ubiposition->x;
							actualposition[1] = ubiposition->y;
							drivenDist=euclideanDistancef(actualposition, initialposition);
							//colision
							if(drivenDist+0.4 < motorDriven){
							doColisionMotion();
							delete initialUbiPosition;
							initialUbiPosition = NULL;
							}
						}
					//	printf("driving with %d\n",motor->calibratedSpeed);
					}
				} else {

					bool seeBlob = false;
					//printf("position: x_%f y_%f phi_%f\n",motor->getPosition()[0],motor->getPosition()[1],motor->getPosition()[2]*180/3.14159265);
					if (!motor->isMoving()) {

						if (detectedBlob != NULL
								&& (isPossible(detectedBlob))) {
							if(wait_no_moving<1){
								//if counts wait_no_moving to null from WAIT_FOR_NO_MOVING
							float measuredCirclePos[4] = { detectedBlob->x,
									detectedBlob->y, detectedBlob->phi,
									detectedBlob->z };
							slamMap->filter(motor->getPosition(),
									measuredCirclePos);
							seeBlob = true;
							}else{
							wait_no_moving -= 1; //deduct from waiting if see for first WAIT_FOR_NO_MOVING after moving
							mapProcedure->wait_stopped += 10; //add to mapping motion to wait longer if robot see something and want stabilized image
							}
						} else {
							slamMap->odometryChange(motor->getPosition());
						}
					} else {
						wait_no_moving = WAIT_FOR_NO_MOVING; //set to max if robot is moving again
						slamMap->odometryChange(motor->getPosition());
					}

					//test whether map is enough sized
					if ((mapProcedure->runs < MINIMAL_RUNS
							|| mapProcedure->wait_stopped > 0) || (!mapProcedure->closedLoop)) {
						if (slamMap->newDetected) {
							slamMap->newDetected = false;
							mapProcedure->wait_stopped += 40;
							sendMap();
						}
						if (slamMap->seeAfterLongTime) {
							slamMap->seeAfterLongTime = false;
							mapProcedure->wait_stopped += 100;
						}

						mapProcedure->doMappingMotion(seeBlob,slamMap);
				//		printf("%d\n", mapProcedure->wait_stopped);
					} else {

						//send map
						actualTask = WAIT;
						motor->setSpeeds(0, 0);

						message_server->sendMessage(MSG_MAP_COMPLETE, NULL, 0);
						std::cout << DEBUGSTRING << " Map sended " << std::endl;
						slamMap->mappingEnded = true;
						slamMap->mergeMap();
						sendMap();
						std::cout << DEBUGSTRING << " Map after merging " << std::endl;
						std::cout << DEBUGSTRING << std::fixed << std::setw( 9 ) << std::setprecision( 7 ) << "Robot position = [ " <<
								slamMap->getRobotPosition()[0] << " | " <<
								slamMap->getRobotPosition()[1] << " | " <<
								slamMap->getRobotPosition()[2] << " ] ";
//						printf("ROBPOS=[ROBPOS [%2.7f ; %2.7f ; %2.7f ; 1 ]]; \n",
//								slamMap->getRobotPosition()[0],
//								slamMap->getRobotPosition()[1],
//								slamMap->getRobotPosition()[0]);
//						printf("ROBPOS=[ROBPOS [%2.7f ; %2.7f ; %2.7f ; 1 ]]; \n",
//								slamMap->getRobotPosition()[0],
//								slamMap->getRobotPosition()[1],
//								slamMap->getRobotPosition()[0]);
						for (int var = 0; var < slamMap->mapSize ; ++var) {
							MappedObjectPosition object = slamMap->getMappedPosition(var);
							std::cout << DEBUGSTRING << std::fixed << std::setw( 9 ) << std::setprecision( 7 ) << "Object position (" <<
									var << ") = [ " <<
									object.xPosition << " | " <<
									object.yPosition << " | " <<
									object.phiPosition << " | " <<
									object.zPosition << " ] " << std::endl;
//							printf("LM%d=[LM%d [%2.7f ; %2.7f ; %2.7f ; %2.7f ]]; \n", var, var,
//									object.xPosition,
//									object.yPosition,
//									object.phiPosition,
//									object.zPosition);
							std::cout << DEBUGSTRING << std::fixed << std::setw( 9 ) << std::setprecision( 7 ) << "Object position uncertainty (" <<
									var << ") = [ " <<
									object.xUncertainty << " | " <<
									object.yUncertainty << " | " <<
									object.phiUncertainty << " | " <<
									object.zUncertainty << " ] " << std::endl;
//
//							printf("LM%dUNCERT=[LM%dUNCERT [%2.7f ; %2.7f ; %2.7f ; %2.7f ]]; \n",
//									var, var, object.xUncertainty,
//									object.yUncertainty,
//									object.phiUncertainty,
//									object.zUncertainty);
						}

						std::cout << DEBUGSTRING << " Map end " << std::endl;
					}
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
	std::cout << DEBUGSTRING << "Exit Mapping" << std::endl;
	return 0;
}
