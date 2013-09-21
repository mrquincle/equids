/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief The total scenario of Grand Challenge 1
 * @file MappingScenario.cpp
 * 
 * Created by Robert Pěnička at CTU in Prague.
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * Copyright (c) 2013 Robert Pěnička <penicrob@fel.cvut.cz>
 *
 * @author    Robert Pěnička
 * @date      Aug 18, 2013
 * @project   Replicator 
 * @company   CTU in Prague
 * @case      Action selection
 */

#include <MappingScenario.h>

#ifdef ENABLE_MAPPING_SCENARIO

#include <messageDataType.h>
#include <wapi/wapi_ubitag.h>
#include <iostream>
#include <inttypes.h>
#include <IRobot.h>
#include <CLeds.h>
#include "../common/cmath.h"

#define VIDEOSTREAM
#define NAME "ActionSelection"

using namespace wapi;

MappingScenario::MappingScenario(CEquids * equids) :
		CScenario(equids) {
	J_CAMERADETECTION = J_POSITION = J_MOTORCALIBRATION = J_MAPPING =
			J_DRIVE_TO_POSITION = J_DOCK_SOCKET = J_ZBMESSENGER =
					J_REMOTE_CONTROL = J_ORGANISM_CONTROL = lastActiveJockey =
							-1;

	quit = false;
	sleepTime = 100000;
	// set initial state
	state = S_START;
}

MappingScenario::~MappingScenario() {

}

bool MappingScenario::Init() {
	bool continue_program = true;

	J_POSITION = equids->find("ubiposition");

	//! Send an error message or also quit program..
	if (J_POSITION == -1) {
		std::cout << "Not defined jockey \"position\"" << std::endl;
		continue_program = false;
	}
	J_ZBMESSENGER = equids->find("zigbeemsg");

	//! Send an error message or also quit program..
	if (J_ZBMESSENGER == -1) {
		std::cout << "Not defined jockey \"zigbeemsg\"" << std::endl;
		continue_program = false;
	}

	J_CAMERADETECTION = equids->find("cameradetection");

	if (J_CAMERADETECTION == -1) {
		std::cout << "Not deffined jockey J_CAMERADETECTION" << std::endl;
		continue_program = false;
	}

	J_MOTORCALIBRATION = equids->find("motorcalibration");

	if (J_MOTORCALIBRATION == -1) {
		std::cout << "Not deffined jockey J_MOTORCALIBRATION" << std::endl;
		continue_program = false;
	}

	J_MAPPING = equids->find("mapping");

	if (J_MAPPING == -1) {
		std::cout << "Not deffined jockey J_MAPPING" << std::endl;
		continue_program = false;
	}

	J_DRIVE_TO_POSITION = equids->find("movetoposition");
	if (J_DRIVE_TO_POSITION == -1) {
		std::cout << "Not deffined jockey J_DRIVE_TO_POSITION" << std::endl;
		continue_program = false;
	}

	J_DOCK_SOCKET = equids->find("docking");
	if (J_DOCK_SOCKET == -1) {
		std::cout << "Not deffined jockey J_DOCK_SOCKET" << std::endl;
		continue_program = false;
	}

	J_REMOTE_CONTROL = equids->find("remotecontrol");
	if (J_REMOTE_CONTROL == -1) {
		std::cout << "Not deffined jockey J_REMOTE_CONTROL" << std::endl;
		continue_program = false;
	}

	J_ORGANISM_CONTROL = equids->find("organismcontrol");
	if (J_ORGANISM_CONTROL == -1) {
		std::cout << "Not deffined jockey J_ORGANISM_CONTROL" << std::endl;
		continue_program = false;
	}

	return continue_program;
}

void MappingScenario::Run() {

	RobotBase::RobotType robot_type;
	RobotBase * robot;
	robot_type = RobotBase::Initialize(NAME);
	robot = RobotBase::Instance();
	for (int i = 0; i < 4; ++i) {
		robot->SetPrintEnabled(i, false);
	}
	CLeds* leds = new CLeds(robot, robot_type);
	leds->color(LC_GREEN);
	printf("setting leds green\n");
	usleep(50000);
	robot->pauseSPI(true);
	while (!robot->isSPIPaused()) {
		usleep(10000);
	}
	//delete leds;
	//delete robot;

	printf("wait for starting message\n");

	while (!quit) {

		CMessage zigbMessage = equids->getJockey(J_ZBMESSENGER)->getMessage();
		//	printf("after read \n");
		//	printf("message type %d \n",zigbMessage.type);
		if (zigbMessage.type != MSG_NONE) {
			//	printf("unpacking \n");
			CMessage unpacked = CMessage::unpackZBMessage(zigbMessage);
			//	printf("unpacked \n");
			switch (unpacked.type) {
			case MSG_MAP_DATA: {
				equids->getJockey(J_MAPPING)->SendMessage(unpacked);
			}
				break;
			case MSG_FORCE_CHANGE_JOCKEY: {
				TState newState;

				memcpy(&newState, unpacked.data, sizeof(TState));
				printf("received MSG_FORCE_CHANGE_JOCKEY %d\n", newState);
				state = newState;
			}
				break;
			default:
				break;
			}
		}

		switch (state) {
		case S_START: {
			state = S_CALIBRATE_ODOMETRY;
			break;

			if (zigbMessage.type == MSG_ZIGBEE_MSG) {
				CMessage normal = CMessage::unpackZBMessage(zigbMessage);
				if (normal.type == MSG_START) {
					printf("starting \n\n\n\n");
					state = S_CALIBRATE_ODOMETRY;
				}
			}

			//state = S_CALIBRATE_ODOMETRY;
			//state = S_DOCK_NOW;
			//state = S_MAPPING_DETECTION;

		}
			;
			break;
		case S_CALIBRATE_ODOMETRY: {
			if (!equids->getJockey(J_MOTORCALIBRATION)->started) {
				if (!equids->getJockey(J_CAMERADETECTION)->started) {
					std::cout << "init Cameradetection" << std::endl;
					equids->initJockey(J_CAMERADETECTION);
#if defined(VIDEOSTREAM)
					equids->getJockey(J_CAMERADETECTION)->acknowledge = 0;
					equids->getJockey(J_CAMERADETECTION)->SendMessage(
							MSG_CAM_VIDEOSTREAM_START, NULL, 0);
					while (equids->getJockey(J_CAMERADETECTION)->acknowledge
							!= 1) {
						usleep(10000);
					}
#endif

				}
				equids->getJockey(J_CAMERADETECTION)->addRedirection(
						J_MOTORCALIBRATION, MSG_CAM_DETECTED_BLOB);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_DETECT_MAPPING, NULL, 0);
				std::cout << "init motor calibration" << std::endl;
				equids->initJockey(J_MOTORCALIBRATION);
				std::cout << "redirecting camera detection to motor calibration"
						<< std::endl;

			}
			//running jockey is J_MOTORCALIBRATION
			CMessage message = equids->getRunningJockey()->getMessage();
			if (message.type == MSG_MOTOR_CALIBRATION_RESULT) {
				std::cout << "calibrated" << std::endl;
				MotorCalibResult calib;
				memcpy(&calib, message.data, sizeof(MotorCalibResult));
				printf("calibration succesfully readed: %e %e %e %d\n",
						calib.odometry_koef1, calib.odometry_koef2,
						calib.odometry_koef3, calib.calibratedSpeed);
				//	equids->getJockey(J_MAPPING)->SendMessage(message);
				equids->sendMessageToALL(MSG_MOTOR_CALIBRATION_RESULT,
						(void*) message.data, message.len);
				std::cout << "remove redirection to motor calibration"
						<< std::endl;
				equids->getJockey(J_CAMERADETECTION)->removeRedirection(
						J_MOTORCALIBRATION, MSG_CAM_DETECTED_BLOB);
				equids->getJockey(J_MOTORCALIBRATION)->stop(true);
				equids->getJockey(J_MOTORCALIBRATION)->quit();
				equids->getJockey(J_CAMERADETECTION)->stop(true);

				std::cout << "motor calibration succesfull" << std::endl;
				if (robot_type == RobotBase::ACTIVEWHEEL) {
					state = S_BUILD_MAP;
				} else {
					state = S_WAIT_FOR_MAP_FROM_OTHERS;
					break;
				}
			}

		}
			;
			break;

		case S_BUILD_MAP: {
			if (!equids->getJockey(J_MAPPING)->started) {
				if (!equids->getJockey(J_CAMERADETECTION)->started) {
					std::cout << "init Cameradetection S_BUILD_MAP"
							<< std::endl;
					equids->initJockey(J_CAMERADETECTION);
#if defined(VIDEOSTREAM)
					equids->getJockey(J_CAMERADETECTION)->acknowledge = 0;
					equids->getJockey(J_CAMERADETECTION)->SendMessage(
							MSG_CAM_VIDEOSTREAM_START, NULL, 0);
					while (equids->getJockey(J_CAMERADETECTION)->acknowledge
							!= 1) {
						usleep(10000);
					}
#endif
				}
				std::cout << "!equids->getJockey(J_POSITION)->started"
						<< std::endl;
				if (!equids->getJockey(J_POSITION)->started) {
					std::cout << "init Ubisence position" << std::endl;
					equids->initJockey(J_POSITION);
				}

				equids->getJockey(J_POSITION)->addRedirection(J_MAPPING,
						MSG_UBISENCE_POSITION);
				equids->getJockey(J_CAMERADETECTION)->addRedirection(J_MAPPING,
						MSG_CAM_DETECTED_BLOB);
				equids->getJockey(J_MAPPING)->addRedirection(J_ZBMESSENGER,
						MSG_ZIGBEE_MSG);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_DETECT_MAPPING, NULL, 0);
				std::cout << "init J_MAPPING" << std::endl;
				equids->initJockey(J_MAPPING);
				std::cout << "redirecting camera detection to J_MAPPING"
						<< std::endl;
			}

			if (zigbMessage.type == MSG_ZIGBEE_MSG) {
				CMessage normalMSG = CMessage::unpackZBMessage(zigbMessage);
				if (normalMSG.type == MSG_MAP_DATA) {
					equids->getJockey(J_MAPPING)->SendMessage(normalMSG);
				}
			}

			//running jockey is J_MAPPING
			CMessage mappingMessage = equids->getRunningJockey()->getMessage();
			if (mappingMessage.type == MSG_MAP_DATA) {
				std::cout << "MAP data.............." << std::endl;

				CMessage packedMessage;
				uint64_t broadcast = Ubitag::BROADCAST;
				printf("broadcas should be: %lld", (long long) broadcast);
				//	printf("broadcas should be: %" PRIu64 "\n", broadcast);
				packedMessage = CMessage::packToZBMessage(broadcast,
						mappingMessage.type, mappingMessage.data,
						mappingMessage.len);
				equids->getJockey(J_ZBMESSENGER)->SendMessage(packedMessage);

			}

			if (mappingMessage.type == MSG_MAP_COMPLETE) {
				std::cout << "MAPPING JOCKEY EXIT" << std::endl;
				equids->getJockey(J_MAPPING)->removeRedirection(J_ZBMESSENGER,
						MSG_ZIGBEE_MSG);
				equids->getJockey(J_POSITION)->removeRedirection(J_MAPPING,
						MSG_UBISENCE_POSITION);
				equids->getJockey(J_CAMERADETECTION)->stop(true);
				equids->getJockey(J_MAPPING)->stop(true);
				lastActiveJockey = J_MAPPING;
				equids->getJockey(J_POSITION)->stop(true);
				state = S_DOCK_SOCKET;
			}
			//	printf("mapping\n");
		}
			break;
		case S_DOCK_SOCKET: {
			//first drive to to front of docking
			if (!equids->getJockey(J_DRIVE_TO_POSITION)->started) {
				//get nearest position of socket from actual ubisence position
				int iter = 0;
				CMessage messageLastPos;
				if (!equids->getJockey(J_POSITION)->started) {
					equids->initJockey(J_POSITION);
				} else {
					equids->getJockey(J_POSITION)->stop(true);
					equids->initJockey(J_POSITION);
				}
				usleep(100000);
				messageLastPos = equids->getJockey(J_POSITION)->getMessage();

				while (messageLastPos.type != MSG_UBISENCE_POSITION && iter < 20) {
					iter += 1;
					usleep(100000);
					messageLastPos =
							equids->getJockey(J_POSITION)->getMessage();
				}

				NearestObjectOfTypeToThisPosition req;
				req.type = DOCK_CIRCLE;
				if (iter < 20) {
					std::cout << "last robot position determined" << std::endl;
					UbiPosition pos;
					memcpy(&pos, messageLastPos.data, sizeof(RobotPosition));
					req.xPosition = pos.x;
					req.yPosition = pos.y;
					req.phiPosition = 0;
				} else {
					std::cout << "can not determine last robot position"
							<< std::endl;
					req.xPosition = 0;
					req.yPosition = 0;
					req.phiPosition = 0;
				}
				equids->getJockey(J_MAPPING)->SendMessage(
						MSG_GET_NEAREST_MAPPED_OBJECT_OF_TYPE_TO_POS, &req,
						sizeof(NearestObjectOfTypeToThisPosition));
				usleep(100000);
				CMessage messageDockPos =
						equids->getJockey(J_MAPPING)->getMessage();
				iter = 0;
				while (messageDockPos.type != MSG_MAP_DATA && iter < 20) {
					iter += 1;
					usleep(100000);
					messageDockPos = equids->getJockey(J_MAPPING)->getMessage();
				}
				RobotPosition driveTo;
				if (iter < 20) {
					MappedObjectPosition dockPos;
					memcpy(&dockPos, messageDockPos.data,
							sizeof(MappedObjectPosition));
					printf("received docking pos %f %f \n", dockPos.xPosition,
							dockPos.yPosition);
					driveTo.phi = dockPos.phiPosition;
					//
					driveTo.phi = normalizeAngle(driveTo.phi);
					float phi_to = dockPos.phiPosition + M_PI;
					if (phi_to > 2 * M_PI) {
						phi_to -= 2 * M_PI;
					}
					float distance_from_dock = 0.3;
					driveTo.x = dockPos.xPosition
							+ distance_from_dock * cos(phi_to);
					driveTo.y = dockPos.yPosition
							+ distance_from_dock * sin(phi_to);
					driveTo.phi = driveTo.phi - 30.0 / 180.0 * M_PI;

					if (!equids->getJockey(J_POSITION)->started) {
						equids->initJockey(J_POSITION);
						printf("starting Jposition\n");
					}else{
						equids->getJockey(J_POSITION)->stop(true);
						equids->initJockey(J_POSITION);
					}
					printf("driving to %f %f %f \n", driveTo.x, driveTo.y,
							driveTo.phi);
					usleep(500000);
					equids->getJockey(J_DRIVE_TO_POSITION)->SendMessage(
							MSG_MOVETOPOSITION, &driveTo,
							sizeof(RobotPosition));
					equids->initJockey(J_DRIVE_TO_POSITION);
					printf("adding redirection to J_DRIVE_TO_POSITION \n");
					equids->getJockey(J_POSITION)->addRedirection(
							J_DRIVE_TO_POSITION, MSG_UBISENCE_POSITION);
				} else {
					//can not determine nearest dock
					std::cout << "can not determine nearest dock" << std::endl;
					state = S_QUIT;
					break;
				}

			}

			CMessage drive_message =
					equids->getJockey(J_DRIVE_TO_POSITION)->getMessage();
			switch (drive_message.type) {
			case MSG_MOVETOPOSITION_DONE: {
				std::cout
						<< "postion reached - received MSG_MOVETOPOSITION_DONE"
						<< std::endl;
				equids->getJockey(J_POSITION)->removeRedirection(
						J_DRIVE_TO_POSITION, MSG_UBISENCE_POSITION);
				std::cout << "stopping ubiposition" << std::endl;
				equids->getJockey(J_POSITION)->stop(true);
				std::cout << "stopping drivetoposition" << std::endl;
				equids->getJockey(J_DRIVE_TO_POSITION)->stop(true);

				if (!equids->getJockey(J_CAMERADETECTION)->started) {
					equids->initJockey(J_CAMERADETECTION);
				}
				equids->getJockey(J_CAMERADETECTION)->addRedirection(
						J_DOCK_SOCKET, MSG_CAM_DETECTED_BLOB_ARRAY);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_DETECT_DOCKING, NULL, 0);
				if (!equids->getJockey(J_DOCK_SOCKET)->started) {
					std::cout << "sending init to J_DOCK_SOCKET" << std::endl;
					equids->initJockey(J_DOCK_SOCKET);
					state = S_DOCK_NOW;
				}
			}
				;
				break;
			case MSG_COLLISION_DETECTED: {
				RobotPosition collisionPos;
				memcpy(&collisionPos, drive_message.data,
						sizeof(RobotPosition));
				printf("collision on %f %f %f\n", collisionPos.x,
						collisionPos.y, collisionPos.phi);
			}
				;
				break;
			default:
				break;
			}

		}
			;
			break;
		case S_DOCKED: {
			printf("docked\n");
			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}
			leds->color(LC_GREEN);
			usleep(500000);
		}
			;
			break;
		case S_DOCK_NOW: {
			if (!equids->getJockey(J_DOCK_SOCKET)->started) {
				if (!equids->getJockey(J_CAMERADETECTION)->started) {
					equids->initJockey(J_CAMERADETECTION);
				}
				equids->getJockey(J_CAMERADETECTION)->addRedirection(
						J_DOCK_SOCKET, MSG_CAM_DETECTED_BLOB_ARRAY);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_DETECT_DOCKING, NULL, 0);
				equids->initJockey(J_DOCK_SOCKET);
			}

			CMessage dockmess = equids->getJockey(J_DOCK_SOCKET)->getMessage();
			if (dockmess.type == MSG_SOCKET_DOCKING_DONE) {
				state = S_DOCKED;
				equids->getJockey(J_DOCK_SOCKET)->stop(true);
				equids->getJockey(J_CAMERADETECTION)->removeRedirection(
						J_DOCK_SOCKET, MSG_CAM_DETECTED_BLOB_ARRAY);
				equids->getJockey(J_CAMERADETECTION)->stop(true);
			}
		}
			;
			break;
		case S_WAIT_FOR_MAP_FROM_OTHERS: {
			int iter;
			if (!equids->getJockey(J_POSITION)->started) {
				equids->initJockey(J_POSITION);
			}else{
				equids->getJockey(J_POSITION)->removeAllRedirections();

			}
			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}
			leds->colorToggle(LC_YELLOW);
			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			CMessage pos_message = equids->getJockey(J_POSITION)->getMessage();
			if (pos_message.type == MSG_UBISENCE_POSITION) {
				UbiPosition posit;
				memcpy(&posit, pos_message.data, pos_message.len);
				NearestObjectOfTypeToThisPosition req;
				req.type = DOCK_CIRCLE;
				req.xPosition = posit.x;
				req.yPosition = posit.y;
				req.phiPosition = 0;
				equids->getJockey(J_MAPPING)->SendMessage(
						MSG_GET_NEAREST_MAPPED_OBJECT_OF_TYPE_TO_POS, &req,
						sizeof(NearestObjectOfTypeToThisPosition));
				usleep(100000);
				CMessage messageDockPos =
						equids->getJockey(J_MAPPING)->getMessage();
				iter = 0;
				while (messageDockPos.type != MSG_MAP_DATA && iter < 5) {
					iter += 1;
					usleep(100000);
					messageDockPos = equids->getJockey(J_MAPPING)->getMessage();
				}
				if(iter<5){
					//succes to get position of nearest dock
					robot->pauseSPI(true);
								while (!robot->isSPIPaused()) {
									usleep(10000);
								}
					state = S_DOCK_SOCKET;
				}
			}

		}
			;
			break;
		case S_STREAM_VIDEO: {

			if (!equids->getJockey(J_CAMERADETECTION)->started) {
				std::cout << "init Cameradetection" << std::endl;
				equids->initJockey(J_CAMERADETECTION);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_VIDEOSTREAM_START, NULL, 0);
			}

			usleep(1000 * 1000 * 60);

		}
			;
			break;

		case S_UBIPOS: {
			if (!equids->getJockey(J_POSITION)->started) {
				equids->initJockey(J_POSITION);
			}
		}
			break;

		case S_MAPPING_DETECTION: {
			if (!equids->getJockey(J_CAMERADETECTION)->started) {
				equids->initJockey(J_CAMERADETECTION);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_DETECT_MAPPING, NULL, 0);
#if defined(VIDEOSTREAM)
				equids->getJockey(J_CAMERADETECTION)->acknowledge = 0;
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_VIDEOSTREAM_START, NULL, 0);
				while (equids->getJockey(J_CAMERADETECTION)->acknowledge != 1) {
					usleep(10000);
				}
#endif
			}
		}
			;
			break;
		case S_REMOTE_CONTROL: {
			if (zigbMessage.type != MSG_NONE) {
				CMessage unpacked = CMessage::unpackZBMessage(zigbMessage);
				if (!equids->getJockey(J_REMOTE_CONTROL)->started) {
					printf("switching to remote controll \n");
					equids->switchToJockey(J_REMOTE_CONTROL);
				}

				if (unpacked.type == MSG_REMOTE_CONTROL) {
					printf("sending MSG_REMOTE_CONTROL \n");
					equids->getJockey(J_REMOTE_CONTROL)->SendMessage(unpacked);
				}
				while (zigbMessage.type != MSG_NONE) {
					zigbMessage =
							equids->getJockey(J_ZBMESSENGER)->getMessage();
				}
			}
		}
			;
			break;
		case S_ORGANISM_REMOTECONTROL: {
			if (!equids->getJockey(J_ORGANISM_CONTROL)->started) {
				IP_rob ipcka[2];
				IP_rob ipScout;
				ipScout.ip2.a = 192;
				ipScout.ip2.b = 168;
				ipScout.ip2.c = 52;
				ipScout.ip2.d = 204;
				IP_rob ipAW;
				ipAW.ip2.a = 192;
				ipAW.ip2.b = 168;
				ipAW.ip2.c = 52;
				ipAW.ip2.d = 65;
				IP_rob ipKB;
				ipKB.ip2.a = 192;
				ipKB.ip2.b = 168;
				ipKB.ip2.c = 52;
				ipKB.ip2.d = 154;
				ipcka[0] = ipScout;
				ipcka[1] = ipAW;
				//ipcka[2] = ipKB;
				char ipchars[20];
				sprintf(ipchars, "%d.%d.%d.%d", ipcka[0].ip2.a, ipcka[0].ip2.b,
						ipcka[0].ip2.c, ipcka[0].ip2.d);
				printf("sending ip %s \n", ipchars);
				sprintf(ipchars, "%d.%d.%d.%d", ipcka[1].ip2.a, ipcka[1].ip2.b,
						ipcka[1].ip2.c, ipcka[1].ip2.d);
				printf("sending ip %s \n", ipchars);
				/*
				 sprintf(ipchars, "%d.%d.%d.%d", ipcka[2].ip2.a, ipcka[2].ip2.b,
				 ipcka[2].ip2.c, ipcka[2].ip2.d);
				 printf("sending ip %s \n", ipchars);
				 */
				std::cout << "sending MSG_INIT_ORGANISM" << std::endl;
				equids->getJockey(J_ORGANISM_CONTROL)->SendMessage(
						MSG_INIT_ORGANISM, ipcka, 2 * sizeof(IP_rob));
				usleep(5000000);
				std::cout << "initJockey" << std::endl;
				equids->initJockey(J_ORGANISM_CONTROL);

			}
			CMessage message =
					equids->getJockey(J_ORGANISM_CONTROL)->getMessage();
			switch (message.type) {
			case MSG_REMOTE_CONTROL: {
				state = S_REMOTE_CONTROLLED_BY_LEADER;
			}
				break;
			case MSG_LEADER: {
				state = S_LEADER_OF_ORGANISM_REMOTECONTROL;
			}
				;
				break;
			default:
				break;
			}
		}
			;
			break;
		case S_ORGANISM_DOCKING: {
			if (!equids->getJockey(J_ORGANISM_CONTROL)->started) {
				IP_rob ipcka[2];
				IP_rob ipScout;
				ipScout.ip2.a = 192;
				ipScout.ip2.b = 168;
				ipScout.ip2.c = 52;
				ipScout.ip2.d = 204;
				IP_rob ipAW;
				ipAW.ip2.a = 192;
				ipAW.ip2.b = 168;
				ipAW.ip2.c = 52;
				ipAW.ip2.d = 65;
				IP_rob ipKB;
				ipKB.ip2.a = 192;
				ipKB.ip2.b = 168;
				ipKB.ip2.c = 52;
				ipKB.ip2.d = 154;
				ipcka[0] = ipScout;
				ipcka[1] = ipAW;
				//ipcka[2] = ipKB;
				char ipchars[20];
				sprintf(ipchars, "%d.%d.%d.%d", ipcka[0].ip2.a, ipcka[0].ip2.b,
						ipcka[0].ip2.c, ipcka[0].ip2.d);
				printf("sending ip %s \n", ipchars);
				sprintf(ipchars, "%d.%d.%d.%d", ipcka[1].ip2.a, ipcka[1].ip2.b,
						ipcka[1].ip2.c, ipcka[1].ip2.d);
				printf("sending ip %s \n", ipchars);
				/*
				 sprintf(ipchars, "%d.%d.%d.%d", ipcka[2].ip2.a, ipcka[2].ip2.b,
				 ipcka[2].ip2.c, ipcka[2].ip2.d);
				 printf("sending ip %s \n", ipchars);
				 */
				std::cout << "sending MSG_INIT_ORGANISM" << std::endl;
				equids->getJockey(J_ORGANISM_CONTROL)->SendMessage(
						MSG_INIT_ORGANISM, ipcka, 2 * sizeof(IP_rob));
				std::cout << "initJockey" << std::endl;
				equids->initJockey(J_ORGANISM_CONTROL);

			}
			CMessage message =
					equids->getJockey(J_ORGANISM_CONTROL)->getMessage();
			switch (message.type) {
			case MSG_REMOTE_CONTROL: {
				state = S_REMOTE_CONTROLLED_BY_LEADER;
			}
				break;
			case MSG_LEADER: {
				state = S_LEADER_OF_ORGANISM_DOCKING;
				if (!equids->getJockey(J_CAMERADETECTION)->started) {
					equids->initJockey(J_CAMERADETECTION);
				}
				equids->getJockey(J_CAMERADETECTION)->addRedirection(
						J_DOCK_SOCKET, MSG_CAM_DETECTED_BLOB_ARRAY);
				equids->getJockey(J_CAMERADETECTION)->SendMessage(
						MSG_CAM_DETECT_DOCKING, NULL, 0);
				equids->getJockey(J_DOCK_SOCKET)->addRedirection(
						J_ORGANISM_CONTROL, MSG_REMOTE_CONTROL);
				equids->getJockey(J_DOCK_SOCKET)->SendMessage(MSG_DOCK_ORGANISM,
						NULL, 0);
				if (!equids->getJockey(J_DOCK_SOCKET)->started) {
					std::cout << "sending init to J_DOCK_SOCKET" << std::endl;
					equids->initJockey(J_DOCK_SOCKET);
				}


			}
				;
				break;
			default:
				break;
			}
		}
			;
			break;
		case S_LEADER_OF_ORGANISM_DOCKING: {
			if (equids->getJockey(J_DOCK_SOCKET)->started) {
				//preforming docking procedure
				CMessage dockMessage =
						equids->getJockey(J_DOCK_SOCKET)->getMessage();
				switch (dockMessage.type) {
				case MSG_SOCKET_DOCKING_DONE:
					state = S_DOCKED;
					break;
				default:
					break;
				}
			}

		}
			;
			break;
		case S_LEADER_OF_ORGANISM_REMOTECONTROL: {
			if (zigbMessage.type != MSG_NONE) {
				CMessage unpacked = CMessage::unpackZBMessage(zigbMessage);

				if (unpacked.type == MSG_REMOTE_CONTROL) {
					printf("sending MSG_REMOTE_CONTROL \n");
					equids->getJockey(J_ORGANISM_CONTROL)->SendMessage(
							unpacked);
				}
			}
		}
			;
			break;
		case S_REMOTE_CONTROLLED_BY_LEADER: {
			if (!equids->getJockey(J_REMOTE_CONTROL)->started) {
				printf("switching to remote controll \n");
				equids->initJockey(J_REMOTE_CONTROL);
				sleepTime = 1000;
			}

			CMessage message =
					equids->getJockey(J_ORGANISM_CONTROL)->getMessage();
			equids->getJockey(J_REMOTE_CONTROL)->SendMessage(message);
		}
			;
			break;
		case S_QUIT:
			quit = true;
			break;
		}
		usleep(sleepTime);
	}
}

#endif //ENABLE_MAPPING_SCENARIO
