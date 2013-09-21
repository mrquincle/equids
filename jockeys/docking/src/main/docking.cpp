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
#include "../motor/CMotors.h"

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

#define DEBUGODOCALIB
#define NAME "Docking"
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "
#define DEBUGSTRING NAME << '[' << getpid() << "] " << __func__ << "(): "
#define PI 3.14159265
#define  APPROACH_LINE 70

using namespace std;
float prumZ = 0;
RobotBase::RobotType robot_type;
RobotBase* robot;
CMessageServer* message_server;
CMessage messagee;
std::string portMS;
DetectedBlob* detectedBlob = NULL;
DetectedBlob* detectedBlobOld = NULL;
int strana = 0;
//bridles
CMotors* motor;
CTimer* timer;
int dx, dy, dphi;
int curvingFactor = 17;
bool hadMoved = false;
bool hadTurned = false;

typedef enum {
	WAIT = 0, SEARCHING, APPROACHING, DOCKING
} DockingStateScout;

typedef enum {
	UNMATCHED = 0, MATCHED
} MatchingState;

typedef enum {
	SWARM = 0, ORGANISM
} Mode;

MatchingState Matching = UNMATCHED;
DockingStateScout DockingState = WAIT;
Mode mode = SWARM;

//actions hadlers
bool stop = false;

void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
//RobotBase::MSPReset();
		exit(0);
	}
}

int sign(int a) {
	if (a > 0)
		return +1;
	if (a < 0)
		return -1;
	return 0;
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
	motor = new CMotors(robot, robot_type);
	std::cout << "init motors" << std::endl;
	motor->init();
	std::cout << "after motor init" << std::endl;
	motor->setSpeeds(0, 0);
	usleep(20000);
}

void turnAW(int a) {
	motor->setSpeeds(0, sign(a) * 30);
	usleep(abs(a) * 45000);
	motor->setSpeeds(0, 0);
	usleep(5000);
	motor->setSpeeds(0, 0);
	usleep(5000);
}

void goAW(int a, int b) {
	//motor->setMotorSpeedsAW(sign(a) * -20, sign(a) * -20, sign(a) * 40);
	motor->setSpeeds(sign(a) * 40, 0);
	//bot->MoveWheelsFront(sign(a)*-20,sign(a)*-20);
	//bot->MoveWheelsRear(sign(a)*40,0);
	usleep(abs(a) * 30000);

	motor->setMotorSpeedsAW(sign(b) * -40, sign(b) * 40, 0);
	// bot->MoveWheelsFront(sign(b)*-40,sign(b)*40);
	// bot->MoveWheelsRear(0,0);
	usleep(abs(b) * 30000);

	motor->setMotorSpeedsAW(0, 0, 0);
	usleep(50000);
}

int dorovnejAW(int a) {

	float alpha = (atan(detectedBlob->y / detectedBlob->x) * 180 / PI) - a;

	if (alpha < -2) {
		turnAW(-(int) floor(alpha));
		fprintf(stdout, "turn o %i (R)\n", (int) ceil(alpha));
		sleep(1);
		return 0;
	} else if (alpha > 2) {
		turnAW(-(int) ceil(alpha));
		fprintf(stdout, "turn o %i (L)\n", (int) floor(alpha));
		sleep(1);
		return 0;
	} else
		return 1;
}

void turnSC(int rot) {
	motor->setSpeeds(0, sign(rot) * 30);
	usleep(32000 * abs(rot)); //24000
	motor->setSpeeds(0, 0);
	usleep(5000);
	motor->setSpeeds(0, 0);
	usleep(5000);
}

void goSC(int x) {
	//x = x/10; //todo rozmer
	//bot->Move(sgn(x)*20,sgn(x)*(-20));
	motor->setMotorSpeedsS(sign(x) * 30, sign(x) * (-30));
	usleep(50000 * abs(x));//40 000
	motor->setSpeeds(0, 0);
	usleep(5000);
	motor->setSpeeds(0, 0);
	usleep(5000);
}

int dorovnejSC(int a) {

	float alpha = (atan(detectedBlob->y / detectedBlob->x) * 180 / PI) + a;

	if (alpha < -2) {
		turnSC((int) ceil(alpha));
		fprintf(stdout, "turn o %i (R)\n", (int) ceil(alpha));
		sleep(2);
		return 0;
	} else if (alpha > 2) {
		turnSC((int) floor(alpha));
		fprintf(stdout, "turn o %i (L)\n", (int) floor(alpha));
		sleep(2);
		return 0;
	} else
		return 1;
}

int dorovnej2SC() {
	if (atan(detectedBlob->y / detectedBlob->x) * 180 / PI > 2) {
		//motor->setSpeeds(0, -30);
		motor->setMotorSpeedsS(30, 30);
		//bot->Move(-30,-30);
		usleep(75000);
		motor->setSpeeds(0, 0);
		usleep(5000);
		motor->setSpeeds(0, 0);
		usleep(5000);
		return 0;
	} else if (atan(detectedBlob->y / detectedBlob->x) * 180 / PI < -2) {
		//motor->setSpeeds(0, 30);
		motor->setMotorSpeedsS(-30, -30);

		//bot->Move(30,30);
		usleep(50000);
		motor->setSpeeds(0, 0);
		usleep(5000);
		motor->setSpeeds(0, 0);
		usleep(5000);
		return 0;
	} else
		return 1;

}

void turnORG(int a) {
	RemoteControlData data;
	data.action = A_NO_ACTION;
	data.speed1 = 0;
	data.speed2 = sign(a) * 40;
	data.directMotorSpeed = false;
	message_server->sendMessage(MSG_REMOTE_CONTROL, &data,
			sizeof(RemoteControlData));
	usleep(abs(a) * 30000);
	data.action = A_NO_ACTION;
	data.speed1 = 0;
	data.speed2 = 0;
	data.directMotorSpeed = false;
	message_server->sendMessage(MSG_REMOTE_CONTROL, &data,
			sizeof(RemoteControlData));
	usleep(50000);
}

void goORG(int a, int b) {
	RemoteControlData data;
	data.action = A_NO_ACTION;
	data.speed1 = sign(a) * 60;
	data.speed2 = 10;
	data.directMotorSpeed = false;
	message_server->sendMessage(MSG_REMOTE_CONTROL, &data,
			sizeof(RemoteControlData));
	usleep(abs(a) * 30000);

	data.speed1 = 0;
	data.speed2 = 0;
	data.speed3 = 0;
	data.directMotorSpeed = true;
	message_server->sendMessage(MSG_REMOTE_CONTROL, &data,
			sizeof(RemoteControlData));
	usleep(50000);

	data.speed1 = -sign(b) * 70;
	data.speed2 = sign(b) * 70;
	data.speed3 = 0;
	data.directMotorSpeed = true;
	message_server->sendMessage(MSG_REMOTE_CONTROL, &data,
			sizeof(RemoteControlData));
	usleep(abs(a) * 30000);
	//motor->setMotorSpeedsAW(sign(b) * -40, sign(b) * 40, 0);

	usleep(abs(b) * 30000);

	data.speed1 = 0;
	data.speed2 = 0;
	data.speed3 = 0;
	data.directMotorSpeed = true;
	message_server->sendMessage(MSG_REMOTE_CONTROL, &data,
			sizeof(RemoteControlData));
	usleep(50000);
}

int dorovnejORG(int a, int b) {
	float alpha = (atan(detectedBlob->y / detectedBlob->x) * 180 / PI) - a;

	if (alpha < -b) {
		turnORG(-(int) floor(alpha));
		fprintf(stdout, "turn o %i (R)\n", (int) ceil(alpha));
		sleep(1);
		return 0;
	} else if (alpha > b) {
		turnORG(-(int) ceil(alpha));
		fprintf(stdout, "turn o %i (L)\n", (int) floor(alpha));
		sleep(1);
		return 0;
	} else
		return 1;

}

void readMessages() {
	//CMessage messagee;

	messagee = message_server->getMessage();
	printf("%i ", messagee.type, " dock \n");
	if (messagee.type != MSG_NONE) {
		switch (messagee.type) {
		case MSG_INIT: {
			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}
			initialize();
			std::cout << "after initialize()" << std::endl;
			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
			//position.type
		}
			;
			break;
		case MSG_DOCK_ORGANISM: {
			mode = ORGANISM;
			sleep(1);
			printf("detected organism\n");
		}
			break;
		case MSG_START: {
			printf("MSG start\n");
			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}

			if (robot_type == RobotBase::ACTIVEWHEEL) {
				ActiveWheel *bot = (ActiveWheel*) robot;
				printf("changing hinge 8.2\n");
				bot->MoveHingeToAngle(8.2);
				for (int i = 0; i < 4; ++i)
					bot->SetPrintEnabled(i, false);
				usleep(500000);
			}
			if (robot_type == RobotBase::SCOUTBOT) {
				ScoutBot *bot = (ScoutBot*) robot;
				usleep(500000);
			}
			if (robot_type == RobotBase::KABOT) {
				KaBot *bot = (KaBot*) robot;
				usleep(500000);
			}
			if (mode == ORGANISM) {
				sleep(1);
				printf("moving hinge and docks\n");
				RemoteControlData remote;
				remote.action = A_MOVE_HINGE;
				remote.hingeAngle = 35;
				message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
						sizeof(RemoteControlData));
				sleep(1);
				for (int i = 0; i < 6; i++) {
					remote.action = A_MOVE_DOCKING;
					remote.dockingAngleLeft = 90;
					remote.dockingAngleRight = 90;
					remote.speed1 = 0;
					remote.speed2 = 0;
					remote.directMotorSpeed = false;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					sleep(2);
				}
			}

			DockingState = SEARCHING;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_STOP: {
			printf("MSG stop\n");
			motor->setSpeeds(0, 0);
			usleep(5000);
			motor->setSpeeds(0, 0);
			usleep(5000);
			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			//actualTask = WAIT;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_QUIT: {
			printf("MSG quit\n");
			{
				motor->setSpeeds(0, 0);
				motor->setSpeeds(0, 0);
				usleep(10000);
				robot->pauseSPI(true);
				//actualTask = WAIT;
				stop = true;
			}
			break;
		}
		case MSG_CAM_DETECTED_BLOB_ARRAY: {

			//printf("MSG detected blob\n");
			if (messagee.len != 0) {
				if (detectedBlob != NULL) {
					if (detectedBlobOld == NULL) {
						detectedBlobOld = new DetectedBlob();
					}
					detectedBlobOld->x = detectedBlob->x;
					detectedBlobOld->y = detectedBlob->y;
					detectedBlobOld->z = detectedBlob->z;
					detectedBlobOld->phi = detectedBlob->phi;
					delete detectedBlob;
				}
				detectedBlob = NULL;
				DetectedBlobWSizeArray* detected = new DetectedBlobWSizeArray();
				memcpy(detected, messagee.data, messagee.len);
//				fprintf(stdout, "pocet tercu: %i\n", detected->size);
//

				if (detected->size == 2) {
					prumZ = (detected->detectedBlobArray[0].z
							+ detected->detectedBlobArray[1].z) / 2;
					printf("prumZ: %f \n", prumZ);

					if (detected->detectedBlobArray[0].z
							> detected->detectedBlobArray[1].z) {
						detectedBlob = new DetectedBlob();
						detectedBlob->x = detected->detectedBlobArray[0].x
								* 1000;
						if (robot_type == RobotBase::ACTIVEWHEEL) {
							detectedBlob->y = detected->detectedBlobArray[0].y
									* 1000 * cos(0.16);
							detectedBlob->z = detected->detectedBlobArray[0].z
									* 1000 * sin(0.16);
							detectedBlob->phi =
									detected->detectedBlobArray[0].phi
											* 180/ PI;
						} else {
							detectedBlob->y = detected->detectedBlobArray[0].y
									* 1000;
							detectedBlob->z = detected->detectedBlobArray[0].z
									* 1000;
							detectedBlob->phi =
									-detected->detectedBlobArray[0].phi
											* 180/ PI;
							if (mode == ORGANISM) {
								detectedBlob->phi = detectedBlob->phi;
								detectedBlob->y = -detectedBlob->y;
							}
						}

					} else {
						detectedBlob = new DetectedBlob();
						detectedBlob->x = detected->detectedBlobArray[1].x
								* 1000;
						if (robot_type == RobotBase::ACTIVEWHEEL) {
							detectedBlob->y = detected->detectedBlobArray[1].y
									* 1000 * cos(0.16);
							detectedBlob->z = detected->detectedBlobArray[1].z
									* 1000 * sin(0.16);
							detectedBlob->phi =
									detected->detectedBlobArray[1].phi
											* 180/ PI;
						} else {
							detectedBlob->y = detected->detectedBlobArray[1].y
									* 1000;
							detectedBlob->z = detected->detectedBlobArray[1].z
									* 1000;
							detectedBlob->phi =
									-detected->detectedBlobArray[1].phi
											* 180/ PI;
							if (mode == ORGANISM) {
								detectedBlob->phi = detectedBlob->phi;
								detectedBlob->y = -detectedBlob->y;
							}
						}

					}
				} else if (detected->size == 1) {

					if (detected->detectedBlobArray[0].z > prumZ - 0.02) {
						detectedBlob = new DetectedBlob();
						detectedBlob->x = detected->detectedBlobArray[0].x
								* 1000;
						if (robot_type == RobotBase::ACTIVEWHEEL) {
							detectedBlob->y = detected->detectedBlobArray[0].y
									* 1000 * cos(0.16);
							detectedBlob->z = detected->detectedBlobArray[0].z
									* 1000 * sin(0.16);
							detectedBlob->phi =
									detected->detectedBlobArray[0].phi
											* 180/ PI;
						} else {
							detectedBlob->y = detected->detectedBlobArray[0].y
									* 1000;
							detectedBlob->z = detected->detectedBlobArray[0].z
									* 1000;
							detectedBlob->phi =
									-detected->detectedBlobArray[0].phi
											* 180/ PI;
							if (mode == ORGANISM) {
								detectedBlob->phi = detectedBlob->phi;
								detectedBlob->y = -detectedBlob->y;
							}
						}

					} else {
						printf("spatny kolecko\n");
						detectedBlob = NULL;
					}
				} else {
					detectedBlob = NULL;
				}
			} else {
				detectedBlob = NULL;
			}

		}
			;
			break;
		default:
			break;
		}
	} else {
//		detectedBlob = NULL; //snad opraveno cteni 2x stejny zpravy
	}

}

int getTargetTurnSC() {
	int s[3];
	int i;
	for (i = 0; i < 3; ++i) {
		readMessages();

		while (detectedBlob == NULL) {
			readMessages();
			if (detectedBlob == NULL) {
				goSC(-20);
				sleep(2);
			}
		}

		s[i] = sign(detectedBlob->phi);
		if (i == 1) {
			turnSC(4);
			sleep(3);

		} else if (i == 2) {
			turnSC(-8);
			sleep(3);
		}
	}
	int soucet = 0;
	int j;
	for (j = 0; j < 3; ++j) {
		soucet += s[j];
		fprintf(stdout, "s[%i] = %i\n", j, s[j]);
	}
	fprintf(stdout, "soucet = %i\n", soucet);
	return sign(soucet);
}

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

	strana = 0;

	while (!stop) {
		switch (DockingState) {
		case (WAIT): {
			readMessages();
			usleep(200000);
		}
			break;
		default: {

			if (hadMoved == true) {
				usleep(500000);
				hadMoved = false;
			}
			if (hadTurned == true) {
				sleep(2);
				hadTurned = false;
			}

			printf("read\n");
			readMessages();
			if (mode == SWARM) {
				switch (robot_type) {
				case RobotBase::ACTIVEWHEEL: {
					if (detectedBlob != NULL)
						printf("detected blob x: %f y: %f phi: %f alpha: %f\n",
								detectedBlob->x, detectedBlob->y,
								detectedBlob->phi,
								atan(detectedBlob->y / detectedBlob->x)
										* 180/ PI);
					if (detectedBlob == NULL && DockingState == SEARCHING) {
						printf("neni nalezen terc\n");
						turnAW(8);
						hadMoved = true;
						Matching = UNMATCHED;
					} else if (detectedBlob != NULL
							&& DockingState == SEARCHING) {
						if (Matching == UNMATCHED) {
							printf("SEARCHING UNMATCHED\n");
							if (dorovnejAW(0) == 1) {
								Matching = MATCHED;
							} else
								sleep(3);


						} else {
							printf("SEARCHING MATCHED\n");
							if (detectedBlob->x > 400) {
								printf("je daleko\n");
								//je daleko
								goAW(((int) detectedBlob->x) - 400, 0);
								Matching = UNMATCHED;
								sleep(1);
							} else if (detectedBlob->x > 70) {
								DockingState = APPROACHING;
							} else {
								DockingState = DOCKING;
							}
						}
					} else if (detectedBlob != NULL
							&& DockingState == APPROACHING) {
						if (abs(
								atan(detectedBlob->y / detectedBlob->x)
										* 180/ PI) < 3) {
							Matching = MATCHED;
						}
						if (Matching == UNMATCHED) {
							printf("APPROACHING UNMATCHED\n");
							if (dorovnejAW(0) == 1) {
								Matching = MATCHED;
							} else
								sleep(1);
						} else {
							printf("APPROACHING MATCHED\n");

							if (detectedBlob->x <= 70) {
								DockingState = DOCKING;
							} else {
								if (detectedBlob->x < 80
										&& detectedBlobOld != NULL
										&& abs(detectedBlob->phi) > 12) {
									if (abs(detectedBlobOld->phi) > 12) {
										fprintf(stdout,
												"uhel je vetsi nez 10\n");
										goAW(-30, 0);
									}
								}
								if (detectedBlob->x > 160) {
									dx = 20;
									dy = 20;
									dphi = 12;
								} else {
									dx = 10;
									dy = 5;
									dphi = 10;//8
									if (abs(detectedBlob->phi) > 15) {
										dy = 15;
									}
								}

								int strana = sign((int) detectedBlob->phi);
								if (abs(detectedBlob->phi) > dphi) {
									fprintf(stdout, "dostrany phi %f \n",
											detectedBlob->phi);
									goAW(0, -dy * strana);
									usleep(500000);

								} else {
									if (detectedBlob->x < 120
											&& abs(detectedBlob->y) > 5) {
										printf("dostrany Y %f \n",
												detectedBlob->y);
										goAW(0,
												-3
														* sign(
																(int) (detectedBlob->y
																		* 1000)));
									} else {
										fprintf(stdout, "dopredu\n");
										goAW(dx, 0);
										sleep(1);
									}
									usleep(800000);
								}
								usleep(500000);
								Matching = UNMATCHED;
								usleep(1000000);
							}
						}
					} else if (detectedBlob == NULL
							&& DockingState == APPROACHING) {
						printf("APPROACHING nenalezeno\n");
						goAW(-3, 0);
						sleep(3);
					} else if (DockingState == DOCKING) {
						printf("docking\n");
//					if(abs(detectedBlob->y) > 1) {
//						printf("dostrany Y dock %i\n", sign((int) (detectedBlob->y* 1000)));
//						goAW(0,3* sign((int) (detectedBlob->y* 1000)));
//					}
						if (detectedBlob != NULL)
							goAW(0, floor((int) detectedBlob->y));
						motor->setMotorSpeedsAW(-45, -45, 90);
						sleep(1);
						motor->setMotorSpeedsAW(-25, -25, 0);
						usleep(500000);
						motor->setMotorSpeedsAW(0, 0, 50);
						usleep(500000);
						motor->setMotorSpeedsAW(0, 0, 0);
						usleep(500000);
						if (((ActiveWheel*) robot)->isEthernetPortConnected(
								ActiveWheel::LEFT)) {
							fprintf(stdout, "propojeno\n");
							message_server->sendMessage(MSG_SOCKET_DOCKING_DONE,
									NULL, 0);
							DockingState = WAIT;
						} else {
							motor->setMotorSpeedsAW(15, 15, -30);
							usleep(500000);
							motor->setMotorSpeedsAW(-35, -35, 70);
							sleep(2);
							motor->setSpeeds(100, 0);
							sleep(1);
							motor->setMotorSpeedsAW(0, 0, 0);
							usleep(6000);
							motor->setMotorSpeedsAW(0, 0, 0);
							sleep(3);
						} //todo
						if (((ActiveWheel*) robot)->isEthernetPortConnected(
								ActiveWheel::LEFT)) {
							fprintf(stdout, "propojeno\n");
							message_server->sendMessage(MSG_SOCKET_DOCKING_DONE,
									NULL, 0);
							DockingState = WAIT;
						} else {
							fprintf(stdout, "nepropojeno\n");
							motor->setSpeeds(-100, 0);
							usleep(100000);
							motor->setSpeeds(0, 0);
							usleep(5000);

							motor->setSpeeds(0, 0);
							usleep(5000);
							goAW(-80, 0);
							DockingState = SEARCHING;
							Matching = UNMATCHED;
							hadMoved = true;
						}
					}
					sleep(1); //1
				}
					break;

				case RobotBase::SCOUTBOT: {
					if (detectedBlob != NULL)
						printf("alpha: %f\n",
								atan2(detectedBlob->y, detectedBlob->x) * 180
										/ M_PI);
//				printf("detected blob x: %f y: %f phi: %f\n", detectedBlob->x,
//						detectedBlob->y, detectedBlob->phi);
//			if (detectedBlobOld != NULL)
//				printf("detected blob  old x: %f y: %f phi: %f\n",
//						detectedBlobOld->x, detectedBlobOld->y,
//						detectedBlobOld->phi);
					if (detectedBlob == NULL && DockingState == SEARCHING) {
						printf("neni nalezen pattern SEARCHING\n");
						if (strana != 0) {
							turnSC(10 * (-strana));
							hadMoved = true;
							sleep(2);
						} else {
							turnSC(5);
							hadMoved = true;
							sleep(1);

						}
						//sleep(1);
					} else if (detectedBlob != NULL
							&& DockingState == SEARCHING) {
						if (Matching == UNMATCHED) {
							printf("SEARCHING UNMATCHED\n");

							if (dorovnejSC(0) == 1) {
								Matching = MATCHED;
							} else {
								sleep(1);
								hadTurned = true;
							}
							sleep(1);
						} else {
							printf("SEARCHING MATCHED\n");
							if (abs(detectedBlob->phi) >= 10) {
								if (strana == 0) {
									strana = getTargetTurnSC();
									hadMoved = true;
								} else {
									strana = sign(detectedBlob->phi);
								}
							} else {
								strana = 0;
							}

							if (detectedBlob->x > 300) {
								//je daleko
								printf("x>300\n");
								goSC((int) detectedBlob->x - 290);
								hadMoved = true;
							} else if (abs(detectedBlob->phi) > 18
									&& detectedBlob->x > 80) {
								//trojuhelnik
								printf("TRIANGLE\n");
								int beta = 90 - (int) abs(detectedBlob->phi);
								int l = (int) abs(
										floor(
												detectedBlob->x * 0.6
														* cos(beta))); //todo
								fprintf(stdout, "\n otocit o: %i\n",
										beta * strana);
								turnSC(beta * strana);
								fprintf(stdout, "\n jedu o: %i \n", l);
								goSC(l);
								turnSC(-beta * strana);
								hadTurned = true;
								sleep(1);
								//strana = -strana;
								//DockingState = SEARCHING;
							} else if (abs(detectedBlob->phi) < 8) {
								printf("STRAIGHT\n");

								dorovnejSC(0);
								if (detectedBlob->x < APPROACH_LINE) {
									dorovnejSC(0);
									DockingState = DOCKING;
								} else {
									goSC(20);
								}
								hadMoved = true;
							} else {
								DockingState = APPROACHING;
								Matching = UNMATCHED;
							}
						}
					} else if (detectedBlob != NULL
							&& DockingState == APPROACHING) {
						//jizda podel kruznice
						if (Matching == UNMATCHED) {
							printf("APPROACHING UNMATCHED\n");
							if (dorovnejSC(strana * curvingFactor) == 1) {
								Matching = MATCHED;
							} else {
								hadMoved = true;

							}
						} else {
							printf("APPROACHING MATCHED\n");
							motor->setSpeeds(40, strana * 10);
							usleep(200000); //100000
							motor->setSpeeds(0, 0);
							usleep(5000);
							motor->setSpeeds(0, 0);
							usleep(5000);
							hadMoved = true;
							Matching = UNMATCHED;
						}
						if (detectedBlob->x < APPROACH_LINE) {
							dorovnejSC(0);
							hadTurned = true;
							Matching = UNMATCHED;
							DockingState = DOCKING;
							hadTurned = true;
						}

					} else if (detectedBlob == NULL
							&& DockingState == APPROACHING) {
						printf("APPROACHING nenalezeno\n");
						turnSC(-strana * 2);
						hadMoved = true;
						sleep(1);

					} else if (detectedBlob != NULL
							&& DockingState == DOCKING) {
						if (Matching == UNMATCHED) {
							printf("DOCKING UNMATCHED\n");

							if (dorovnej2SC() == 1) { //2sc
								Matching = MATCHED;
							} else {
								sleep(2);
								hadTurned = true;
							}
						} else {
							printf("DOCKING MATCHED\n");

							if (detectedBlobOld != NULL
									&& abs(detectedBlob->phi) > 13) {
								if (abs(detectedBlobOld->phi) > 13) {
									fprintf(stdout,
											"uhel je vetsi nez 10 new: %f old: %f\n",
											detectedBlob->phi,
											detectedBlobOld->phi);
									goSC(-100);
									hadMoved = true;
									Matching == UNMATCHED;
									DockingState = SEARCHING; //SEARCH_FOR_PATTERN;
									sleep(1);
								}
							} else if (detectedBlob->x < 50) {
								motor->setSpeeds(40, 0);
								sleep(2);
								motor->setSpeeds(-30, 0);
								sleep(1);
								motor->setSpeeds(60, 0);
								sleep(1);
								//todo kontrola pripojeni

								motor->setSpeeds(0, 0);
								usleep(5000);
								motor->setSpeeds(0, 0);
								sleep(3);
								if (((ScoutBot*) robot)->isEthernetPortConnected(
										ScoutBot::FRONT)) {
									fprintf(stdout, "propojeno\n");

									message_server->sendMessage(
											MSG_SOCKET_DOCKING_DONE, NULL, 0);
									DockingState = WAIT;
								} else {
									fprintf(stdout, "nepropojeno\n");
									goSC(-120);
									curvingFactor = 12;
									strana = 0;
									Matching = UNMATCHED;
									DockingState = SEARCHING;

								}
								hadMoved = true;

							} else {
								motor->setSpeeds(30, 0);
								usleep(100000);
								motor->setSpeeds(0, 0);
								usleep(5000);
								motor->setSpeeds(0, 0);
								usleep(5000);
								//sleep(1);
								hadMoved = true;
								Matching = UNMATCHED;
							}

						}

					} else if (detectedBlob == NULL
							&& DockingState == DOCKING) {
						printf("nevidim pattern DOCKING \n");
//				if(strana==0){
						motor->setSpeeds(-30, 0);
//				}else{
//					motor->setSpeeds(0,-strana*30);
//				}
						usleep(40000);
						motor->setSpeeds(0, 0);
						usleep(5000);
						motor->setSpeeds(0, 0);
						sleep(2);
						hadMoved = true;

					}
				}
					break;
//			case RobotBase::KABOT: {
//				printf("jsem KABOT\n");
//				if (detectedBlob != NULL)
//					printf("detected blob x: %f y: %f phi: %f\n",
//							detectedBlob->x, detectedBlob->y,
//							detectedBlob->phi);
//				if (detectedBlob == NULL && DockingState == SEARCHING) {
//					printf("neni nalezen terc\n");
//					turnKB(8);
//					hadMoved = true;
//					Matching = UNMATCHED;
//					if (detectedBlob != NULL)
//						printf("detected blob x: %f y: %f phi: %f\n",
//								detectedBlob->x, detectedBlob->y,
//								detectedBlob->phi);
//
//				} else if (detectedBlob != NULL && DockingState == SEARCHING) {
//					if (Matching == UNMATCHED) {
//						printf("SEARCHING UNMATCHED\n");
//						if (dorovnejKB(0) == 1) {
//							Matching = MATCHED;
//						} else
//							sleep(2);
//
//					} else {
//						printf("SEARCHING MATCHED\n");
//						if (detectedBlob->x > 400) {
//							printf("je daleko\n");
//							//je daleko
//							goKB(((int) detectedBlob->x) - 400, 0);
//							Matching = UNMATCHED;
//							sleep(1);
//						} else if (detectedBlob->x > 60) {
//							DockingState = APPROACHING;
//						} else {
//							DockingState = DOCKING;
//						}
//					}
//				}
//			}
//				break;
				}
			} else { //organism mod

				RemoteControlData remote;
//				remote.action = A_MOVE_DOCKING;
//				remote.dockingAngleLeft = 90;
//				remote.dockingAngleRight = 90;
//				message_server->sendMessage(MSG_REMOTE_CONTROL,&remote,sizeof(RemoteControlData));

//				remote.action = A_NO_ACTION;
//				remote.speed1 = 50;
//				remote.speed2 = 20; //curving
//				remote.directMotorSpeed = false;
//				message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
//						sizeof(RemoteControlData));
//
//				sleep(1);
//				remote.action = A_NO_ACTION;
//				remote.speed1 = 0;
//				remote.speed2 = 0; //curving
//				remote.directMotorSpeed = false;
//				message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
//						sizeof(RemoteControlData));

				if (detectedBlob != NULL)
					printf("detected blob x: %f y: %f phi: %f alpha: %f\n",
							detectedBlob->x, detectedBlob->y, detectedBlob->phi,
							atan(detectedBlob->y / detectedBlob->x) * 180 / PI);
				if (detectedBlob == NULL && DockingState == SEARCHING) {
					printf("neni nalezen terc\n");
					turnORG(8);
					hadMoved = true;
					Matching = UNMATCHED;
				} else if (detectedBlob != NULL && DockingState == SEARCHING) {
					if (Matching == UNMATCHED) {
						printf("SEARCHING UNMATCHED\n");
						if (dorovnejORG(0, 5) == 1) {
							Matching = MATCHED;
						} else
							sleep(2);

					} else {
						printf("SEARCHING MATCHED\n");
						if (detectedBlob->x > 400) {
							printf("je daleko\n");
							//je daleko
							goORG(((int) detectedBlob->x) - 400, 0);
							Matching = UNMATCHED;
							sleep(1);
						} else if (detectedBlob->x > 55) {
							DockingState = APPROACHING;
						} else {
							DockingState = DOCKING;
						}
					}
				} else if (detectedBlob != NULL
						&& DockingState == APPROACHING) {
					if (abs(atan(detectedBlob->y / detectedBlob->x) * 180 / PI)
							< 3) {
						Matching = MATCHED;
					}
					if (Matching == UNMATCHED) {
						printf("APPROACHING UNMATCHED\n");
						if (dorovnejORG(0, 5) == 1) {
							Matching = MATCHED;
						} else
							sleep(1);
					} else {
						printf("APPROACHING MATCHED\n");

						if (detectedBlob->x <= 70) {
							DockingState = DOCKING;
						} else {
							if (detectedBlob->x < 80 && detectedBlobOld != NULL
									&& abs(detectedBlob->phi) > 10) {
								if (abs(detectedBlobOld->phi) > 10) {
									fprintf(stdout, "uhel je vetsi nez 10\n");
									goORG(-30, 0);
								}
							}
							if (detectedBlob->x > 200) {
								dx = 20;
								dy = 20;
								dphi = 12;
							} else {
								dx = 10;
								dy = 5;
								dphi = 8;//8
								if (abs(detectedBlob->phi) > 15) {
									dy = 15;
								}
								if (abs(detectedBlob->phi) > 15) {
									dy = 15;
								}
							}

							int strana = sign((int) detectedBlob->phi);
							if (abs(detectedBlob->phi) > dphi) {
								fprintf(stdout, "dostrany phi %f \n",
										detectedBlob->phi);
								goORG(0, -dy * strana);
								usleep(500000);

							} else {
//								if (detectedBlob->x < 120
//										&& abs(detectedBlob->y) > 5) {
////									printf("dostrany Y %i \n",
////											sign(
////													(int) (detectedBlob->y
////															* 1000)));
////									goORG(0,
////											3
////													* sign(
////															(int) (detectedBlob->y
////																	* 1000)));
//								} else {
								fprintf(stdout, "dopredu\n");
								goORG(dx, 0);
//								}
								usleep(800000);
							}
							usleep(500000);
							Matching = UNMATCHED;
							usleep(1000000);
						}
					}
				} else if (detectedBlob == NULL
						&& DockingState == APPROACHING) {
					printf("APPROACHING nenalezeno\n");
					goORG(-3, 0);
					sleep(3);
				} else if (DockingState == DOCKING) {
					printf("docking\n");
					//					if(abs(detectedBlob->y) > 1) {
					//						printf("dostrany Y dock %i\n", sign((int) (detectedBlob->y* 1000)));
					//						goAW(0,3* sign((int) (detectedBlob->y* 1000)));
					//					}
//					if (detectedBlob != NULL)
//						goORG(0, floor((int) detectedBlob->y));

					remote.action = A_NO_ACTION;
					remote.speed1 = -60;
					remote.speed2 = -60;
					remote.speed3 = 90;
					remote.directMotorSpeed = true;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					sleep(1);
					remote.action = A_NO_ACTION;
					remote.speed1 = -60;
					remote.speed2 = -60;
					remote.speed3 = 90;
					remote.directMotorSpeed = true;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					sleep(1);
					goORG(30, 0);
					sleep(1);
					remote.action = A_NO_ACTION;
					remote.speed1 = 30;
					remote.speed2 = 30;
					remote.speed3 = 0;
					remote.directMotorSpeed = true;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					usleep(500000);
					remote.action = A_NO_ACTION;
					remote.speed1 = -60;
					remote.speed2 = -60;
					remote.speed3 = 90;
					remote.directMotorSpeed = true;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					sleep(1);

					remote.action = A_NO_ACTION;
					remote.speed1 = -30;
					remote.speed2 = -30;
					remote.speed3 = 0;
					remote.directMotorSpeed = true;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					usleep(700000);
					remote.action = A_NO_ACTION;
					remote.speed1 = -60;
					remote.speed2 = -60;
					remote.speed3 = 90;
					remote.directMotorSpeed = true;
					message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
							sizeof(RemoteControlData));
					sleep(1);
					if (((ScoutBot*) robot)->isEthernetPortConnected(
							ScoutBot::FRONT)) {
						fprintf(stdout, "propojeno\n");
						message_server->sendMessage(MSG_SOCKET_DOCKING_DONE,
								NULL, 0);
						DockingState = WAIT;
					} else {
						remote.action = A_NO_ACTION;
						remote.speed1 = 18;
						remote.speed2 = 18;
						remote.speed3 = -30;
						remote.directMotorSpeed = true;
						message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
								sizeof(RemoteControlData));
						sleep(1);
						usleep(500000);
						remote.action = A_NO_ACTION;
						remote.speed1 = -40;
						remote.speed2 = -40;
						remote.speed3 = 70;
						remote.directMotorSpeed = true;
						message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
								sizeof(RemoteControlData));
						sleep(1);
						remote.action = A_NO_ACTION;
						remote.speed1 = -40;
						remote.speed2 = -40;
						remote.speed3 = 70;
						remote.directMotorSpeed = true;
						message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
								sizeof(RemoteControlData));
						sleep(1);
						remote.action = A_NO_ACTION;
						remote.speed1 = 0;
						remote.speed2 = 0;
						remote.speed3 = 0;
						remote.directMotorSpeed = true;
						message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
								sizeof(RemoteControlData));
						usleep(500000);
						sleep(4);
					}
					if (((ScoutBot*) robot)->isEthernetPortConnected(
							ScoutBot::FRONT)) {
						fprintf(stdout, "propojeno\n");
						message_server->sendMessage(MSG_SOCKET_DOCKING_DONE,
								NULL, 0);
						DockingState = WAIT;
					} else {

						fprintf(stdout, "nepropojeno\n");
						remote.action = A_NO_ACTION;
						remote.speed1 = -80;
						remote.speed2 = 0;
						remote.directMotorSpeed = false;
						message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
								sizeof(RemoteControlData));
						usleep(400000);
						remote.action = A_NO_ACTION;
						remote.speed1 = 0;
						remote.speed2 = 0;
						remote.directMotorSpeed = false;
						message_server->sendMessage(MSG_REMOTE_CONTROL, &remote,
								sizeof(RemoteControlData));
						usleep(5000);
						goORG(-80, 0);
						goORG(-80, 0);
						goORG(-80, 0);

						DockingState = SEARCHING;
						Matching = UNMATCHED;
						hadMoved = true;
					}
				}
				sleep(1);

			}
			sleep(1);
			if (detectedBlob == NULL)
				sleep(1);

		}

			break;
		}
	}

	//motor->setMotorPosition(0, 0, 0);

	motor->setSpeeds(0, 0);
	usleep(5000);
	motor->setSpeeds(0, 0);
	usleep(5000);
	delete motor;
	delete message_server;
	return 0;
}
