/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief The total scenario of Grand Challenge 1
 * @file GrandChallenge1Scenario.cpp
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

#include <GrandChallenge1Scenario.h>

#include <wapi/wapi_ubitag.h>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <iomanip>
#include <messageDataType.h>
#include <IRobot.h>
#include <CLeds.h>
//#define VIDEOSTREAM

//#define LASERSCAN_VIDEOSTREAM // disabled in grand challenge because it conflicts with camera detection
//#define __START_WITH_ZIGBEE__


//! Name of this challenge
static const std::string NAME = "GrandChallenge1";
static uint8_t recruiting_tmp[8];
static uint8_t recruiting_answ[64];
static uint8_t recruiting_accp[64];
static struct RobotPosition goal;

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "
#define sqr(x) (x)*(x)
GrandChallenge1Scenario::GrandChallenge1Scenario(CEquids * equids) :
		CScenario(equids) {
	J_MAPPING = J_LASER_RECOGNITION = J_VISUAL_EXPLORATION =
			J_INFRARED_EXPLORATION = J_WENGUO = J_CAMERADETECTION = J_ZBMESSENGER = J_POSITION = J_MOVETO = -1;

	quit = false;

	// set initial state
	state = S_START;
}

GrandChallenge1Scenario::~GrandChallenge1Scenario() {

}

bool GrandChallenge1Scenario::Init() {
	bool continue_program = true;

	int id = V_MAPPING;
	J_MAPPING = equids->find("mapping", id);
	id = V_WENGUO;
	J_WENGUO = equids->find("wenguo", id);
	id = V_LASERSCAN;
	J_LASER_RECOGNITION = equids->find("laser", id);
	id = V_AVOIDIR;
	J_INFRARED_EXPLORATION = equids->find("avoidir", id);
	id = V_ZIGBEEMSG;
	J_ZBMESSENGER = equids->find("zigbeemsg", id);
	id = V_CAMERADETECTION;
	J_CAMERADETECTION = equids->find("cameradetection", id);
	id = V_UBIPOSITION;
	J_POSITION = equids->find("ubiposition", id);
   id = V_MOVE_TO_POSITION;
   J_MOVETO = equids->find("moveto", id);
	id = V_REMOTECONTROL;
	J_REMOTE_CONTROL = equids->find("remotecontrol",id);

	//! Send an error message or also quit program..
	if (J_MAPPING == -1) {
		std::cout << DEBUG << "Not defined jockey \"mapping\"" << std::endl;
		continue_program = true;
	}
   if (J_MOVETO == -1) {
      std::cout << DEBUG << "Not defined jockey \"moveto\"" << std::endl;
      continue_program = false;
   }
	
	if (J_WENGUO == -1) {
			std::cout << "Not defined jockey \"wenguo\"" << std::endl;
			continue_program = true;
	}
	if (J_LASER_RECOGNITION == -1) {
		std::cout << DEBUG << "Not defined jockey \"laser\"" << std::endl;
		continue_program = false;
	}

	if (J_INFRARED_EXPLORATION == -1) {
		std::cout << DEBUG << "Not defined jockey \"avoidir\"" << std::endl;
		continue_program = false;
	}

	if (J_ZBMESSENGER == -1) {
		std::cout << DEBUG << "Not defined jockey \"zigbeemsg\"" << std::endl;
		continue_program = false;
	}

	if (J_CAMERADETECTION == -1) {
		std::cout << DEBUG << "Not defined jockey \"cameradetection\"" << std::endl;
		continue_program = false;
	}

	if (J_POSITION == -1) {
		std::cout << DEBUG << "Not defined jockey \"ubiposition\"" << std::endl;
		continue_program = false;
	}



	return continue_program;
}

static int recr_state;   
static int robot_nearest_id;
static float robot_nearest_distance;

void init_recruiting() {
   recr_state= 0;
   robot_nearest_id = -1;
}

void GrandChallenge1Scenario::Run() {
   int robot_id = -1;
   int robot_need_id = -1;
   int robot_need_time;
   int robot_recr_time;
   UbiPosition goto_pos;
   char* robotID = getenv("sr_id");
   init_recruiting();
   
   if (robotID != NULL) {
      robot_id = atoi(robotID);
   }

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
   
	// eugen's number is +4917682084745.

	std::cout << "################################################################################" << std::endl;
	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;
	std::cout << "################################################################################" << std::endl;

	bool turn_cam_continuously_on_sent = false;

	leds->color(LC_GREEN);
	std::cerr << DEBUG << "Green leds on start-up" << std::endl;

	while (!quit) {

		if (!equids->getJockey(J_ZBMESSENGER)->started) {
			std::cerr << DEBUG << "The ZigBee jockey is not running, that cannot be correct" << std::endl;
		}

		//receive messages over zigbee
		CMessage zigbMessage = equids->getJockey(J_ZBMESSENGER)->getMessage();
		if (zigbMessage.type != MSG_NONE) {
			CMessage unpacked = CMessage::unpackZBMessage(zigbMessage);
			switch (unpacked.type) {
			case MSG_MAP_DATA: {
				std::cout << DEBUG << "Received message \"" << StrMessage[zigbMessage.type] << "\"" << std::endl;
				equids->getJockey(J_MAPPING)->SendMessage(unpacked);
            break;
			}
			case MSG_ACTIVE_JOCKEYS: {
				std::cout << DEBUG << "received message \"" << StrMessage[zigbMessage.type] << "\"" << std::endl;
				// send message back
				std::vector<vocab_t> jockey_ids;
				jockey_ids.clear();
				equids->getAllRunningJockeys(jockey_ids);
				int id = V_ACTIONSELECTION;
				jockey_ids.push_back(id);
				int *jockey_arr = NULL;
				int n_jockeys = jockey_ids.size();
				int len = n_jockeys * ( sizeof(int) / sizeof(char) );
				jockey_arr = new int[n_jockeys];
				for (int i = 0; i < n_jockeys; i++) {
					jockey_arr[i] = jockey_ids[i];
//				if (len > 0) {
//					jockey_arr = &jockey_ids[0];
				}
				CMessage packedMessage = CMessage::packToZBMessage(-1, MSG_ACTIVE_JOCKEYS, jockey_arr, len);
				std::cout << DEBUG << "????????????????????????????????????????????????????????????????????????????????????????????????????????????" << std::endl;
				std::cout << DEBUG << "respond with message indicating " << n_jockeys << " active jockeys, namely: ";
				for (int i = 0; i < n_jockeys; i++) std::cout << jockey_arr[i] << ' ';
				std::cout << std::endl;
				std::cout << DEBUG << "ActionSelection " << id << " should be one of them" << std::endl;
//				equids->getJockey(J_ZBMESSENGER)->acknowledge = 0;
				equids->getJockey(J_ZBMESSENGER)->SendMessage(packedMessage);
//				while (equids->getJockey(J_ZBMESSENGER)->acknowledge != 1) { usleep(10000); }
            break;
			}
         case MSG_FORCE_CHANGE_JOCKEY: {
            TState newState;
            memcpy(&newState, unpacked.data, sizeof(TState));
            std::cout << DEBUG << "received MSG_FORCE_CHANGE_JOCKEY" << (int)newState << std::endl;
            state = newState;
            break;
         }
         case MSG_NEED_ORG: {
            if (robot_type==(int)unpacked.data[0]) {
               printf("SOMEBODY NEED Me, I'll send my position\n");
               if (robot_need_id<0 || robot_need_time<time(NULL)-10) {
                  robot_need_id = (int)unpacked.data[1];
                  robot_need_time = time(NULL);
                  CMessage recruitingMsg;
                  CMessage packedMessage;

                  recruitingMsg.type = MSG_HELP_ORG;
                  recruiting_answ[0] = (uint8_t)robot_id;
                  memcpy(&recruiting_answ[1], &equids->getJockey(J_POSITION)->actual_position, sizeof(struct UbiPosition));
                  recruitingMsg.data = recruiting_answ;
                  recruitingMsg.len=1+sizeof(struct UbiPosition); 
            
                  packedMessage = CMessage::packToZBMessage(robot_need_id,
                     recruitingMsg.type, recruitingMsg.data,
                     recruitingMsg.len);
                  equids->getJockey(J_ZBMESSENGER)->SendMessage(packedMessage);
               } else {
                  printf("I promise to help %i in time %i now is %i\n", robot_need_id, robot_need_time, time(NULL));
               }
            }
            break;
         }
         case MSG_HELP_ORG: {
            printf("ROBOT from %f, %f wants to help\n");
            UbiPosition *pos=(UbiPosition *)&unpacked.data[1];
            UbiPosition *my = &equids->getJockey(J_POSITION)->actual_position;
            float robot_distance=sqrt(sqr(pos->x-my->x)+sqr(pos->y-my->y));
            if (robot_nearest_id<0 || robot_nearest_distance>robot_distance) {
               robot_nearest_id = (int)unpacked.data[0];
               robot_nearest_distance = robot_distance;
            }
            break;
         }
         case MSG_HELP_ACP: {
            UbiPosition *pos=(UbiPosition *)&unpacked.data[0];
            UbiPosition *my = &equids->getJockey(J_POSITION)->actual_position;
            float robot_distance=sqrt(sqr(pos->x-my->x)+sqr(pos->y-my->y));
            goal.x = pos->x + 0.2*(my->x-pos->x)/robot_distance;
            goal.y = pos->y + 0.2*(my->y-pos->y)/robot_distance;
            goal.phi = atan2(pos->y-my->y,pos->x-my->x);
            printf("MOVE TO pos (%f, %f, %f)\n", goal.x, goal.y, goal.phi);
            CMessage go_to;
            go_to.type = MSG_MOVETOPOSITION;
            go_to.data = (uint8_t *)&goal;
            go_to.len = sizeof(RobotPosition);
            equids->getJockey(J_MOVETO)->SendMessage(go_to);
            equids->switchToJockey(J_MOVETO);
            state = S_CONNECTING;            
				break;
			}
			default: {
				std::cout << DEBUG << "received zigbee unknown message " << (int)unpacked.type << std::endl;
            break;
			}
			}
		}

		switch (state) {
		case S_START: {
			if (!equids->getJockey(J_ZBMESSENGER)->started) {
				equids->initJockey(J_ZBMESSENGER, true);
			}
#ifdef __START_WITH_ZIGBEE__
			if (zigbMessage.type != MSG_NONE) {
				CMessage unpacked = CMessage::unpackZBMessage(zigbMessage);
				switch (unpacked.type) {
				case MSG_START: {
					std::cout << DEBUG << "Go to state mapping through MSG_START through zigbee" << std::endl;
					state = S_MAPPING;
					break;
				}
				default : break;
				}
			}
#else
			state = S_MAPPING;
#endif



			// for the scout, the mapping daemon will directly continue with S_EXPLORATION

			break;
		}
		case S_REMOTE_CONTROL: {
					if (zigbMessage.type != MSG_NONE) {
						CMessage unpacked = CMessage::unpackZBMessage(zigbMessage);
						if (!equids->getJockey(J_REMOTE_CONTROL)->started) {
							printf("switching to remote controll \n");
							equids->switchToJockey(J_REMOTE_CONTROL);
						}
						//sleepTime = 50;
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
		case S_MAPPING: {
			if (!equids->getJockey(J_MAPPING)->started) {
				if (!equids->getJockey(J_CAMERADETECTION)->started) {
					std::cout << "init Cameradetection S_BUILD_MAP"
							<< std::endl;
					equids->initJockey(J_CAMERADETECTION,true);
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
				std::cout << DEBUG << "!equids->getJockey(J_POSITION)->started"
						<< std::endl;
				if (!equids->getJockey(J_POSITION)->started) {
					std::cout << DEBUG << "init Ubisence position" << std::endl;
					equids->initJockey(J_POSITION,true);
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
				equids->switchToJockey(J_MAPPING);
				std::cout << DEBUG << "redirecting camera detection to J_MAPPING"
						<< std::endl;
			}

			//running jockey is J_MAPPING
			CMessage mappingMessage = equids->getRunningJockey()->getMessage();

			if (mappingMessage.type == MSG_MAP_DATA) {
				std::cout << DEBUG << "MAP data.............." << std::endl;

				CMessage packedMessage;

				packedMessage = CMessage::packToZBMessage(-1,
						mappingMessage.type, mappingMessage.data,
						mappingMessage.len);
				equids->getJockey(J_ZBMESSENGER)->SendMessage(packedMessage);

			}

			if (mappingMessage.type == MSG_MAP_COMPLETE) {
				std::cout << DEBUG << "MAPPING JOCKEY EXIT" << std::endl;
				equids->getJockey(J_POSITION)->removeAllRedirections();
				equids->getJockey(J_CAMERADETECTION)->stop(true);
				equids->getJockey(J_MAPPING)->stop(true);
				equids->getJockey(J_POSITION)->stop(true); // actually not necessary?
				state = S_EXPLORATION;
			}
		}
		;
		break;
		case S_EXPLORATION: {
			if (!equids->getJockey(J_INFRARED_EXPLORATION)->started) {
				equids->switchToJockey(J_INFRARED_EXPLORATION);
			}
			CMessage message = equids->getMessage(J_INFRARED_EXPLORATION);
			if (message.type == MSG_COLLISION_DETECTED) {
				std::cout << DEBUG << "Got message: " << StrMessage[message.type] << std::endl;
				state = S_DETECT_OBJECT;
			} else if (message.type != MSG_NONE) {
				std::cout << DEBUG << "Didn't expect message "
						<< StrMessage[message.type] << " from infrared "
						<< std::endl;
			}
			break;
		}
		case S_DETECT_OBJECT: {
			if (equids->getJockey(J_CAMERADETECTION)->started) {
				equids->getJockey(J_CAMERADETECTION)->stop(true);
			}
			if (!equids->getJockey(J_LASER_RECOGNITION)->started) {
				equids->initJockey(J_POSITION,true);
				equids->getJockey(J_POSITION)->removeAllRedirections();
				// from ubisense positions to the laser scan application
				equids->getJockey(J_POSITION)->addRedirection(J_LASER_RECOGNITION,MSG_UBISENCE_POSITION);
				equids->switchToJockey(J_LASER_RECOGNITION);
			}
#ifdef LASERSCAN_VIDEOSTREAM
			if (!turn_cam_continuously_on_sent) {
				CMessage msg;
				msg.len = 1;
				msg.data = new uint8_t[msg.len];
				msg.data[0] = (uint8_t)true;
				msg.type = MSG_CAM_TURN_ON_ALL_THE_TIME;
				std::cout << DEBUG << "Send message: " << StrMessage[msg.type] << std::endl;
				equids->sendMessage(J_LASER_RECOGNITION, msg);
				turn_cam_continuously_on_sent = true;
			}
#endif
			CMessage msg;
			CreateMsgLaserDetectObject(msg);
			equids->sendMessage(J_LASER_RECOGNITION, msg);
			std::cout << DEBUG << "Recognition message sent" << std::endl;
			state = S_OBJECT_DETECTED;
			break;
		}
		case S_OBJECT_DETECTED: {
			CMessage message = equids->getMessage(J_LASER_RECOGNITION);
			if (message.type == MSG_MAP_DATA) {
				std::cout << DEBUG << "Got message: " << StrMessage[message.type]
				                                                    << std::endl;
				int len = sizeof(struct MappedObjectPosition);
				if (message.len != len) {
					std::cerr << DEBUG
							<< "Error, expected payload of MappedObjectPosition of size "
							<< len << " while it is " << message.len
							<< std::endl;
				}
				MappedObjectPosition positionForMappedObject;
				memcpy(&positionForMappedObject, message.data, message.len);
				std::cout << DEBUG << "Detected object: "
						<< StrMapObjectType[positionForMappedObject.type]
						                    << std::endl;

				switch (positionForMappedObject.type) {
				case SMALL_STEP:
				case LARGE_STEP:
					state = S_RECRUITING;
               std::cout << DEBUG << "Start recruiting robots for organism and update map with info on \"step\"" << std::endl;
               equids->getJockey(J_MAPPING)->SendMessage(message);
               if (J_WENGUO>=0) {
                  uint8_t cmd_data[3];
                  equids->switchToJockey(J_WENGUO);
                  cmd_data[0] = 2; //recruiting side: 0 -- front, 1 -- left, 2 -- back, 3 -- right
                  cmd_data[1] = 3; //recruited robot type: 1 -- KIT, 2 -- Scout, 3 -- ActiveWheel
                  cmd_data[2] = 1; //recruited robot side: 0 -- front, 1 -- left, 2 -- back, 3 -- right
                  equids->getJockey(J_WENGUO)->SendMessage(DAEMON_MSG_RECRUITING, cmd_data, sizeof(cmd_data));
               }
					break;
				case WALL:
					std::cout << DEBUG << "Go back to exploration, and update map with info on \"wall\"" << std::endl;
					equids->getJockey(J_POSITION)->stop(true);
					state = S_EXPLORATION;
					equids->getJockey(J_MAPPING)->SendMessage(message);
					break;
				default:
					std::cout << DEBUG << "Go back to exploration" << std::endl;
					equids->getJockey(J_POSITION)->stop(true);
					state = S_EXPLORATION;
				}
			} else if (message.type != MSG_NONE) {
				std::cout << DEBUG << "Didn't expect message "
						<< StrMessage[message.type] << " from laser "
						<< std::endl;
			}
			break;
		}
      case S_RECRUITING: {
         CMessage recruitingMsg;
         CMessage packedMessage;
         if (recr_state==0) {
            recruitingMsg.type = MSG_NEED_ORG;
            recruiting_tmp[0] = (uint8_t)(RobotBase::ACTIVEWHEEL);
            recruiting_tmp[1] = (uint8_t)robot_id;
            recruitingMsg.data = recruiting_tmp;
            recruitingMsg.len=2;
            
            packedMessage = CMessage::packToZBMessage(-1,
                  recruitingMsg.type, recruitingMsg.data,
                  recruitingMsg.len);
            equids->getJockey(J_ZBMESSENGER)->SendMessage(packedMessage);
            recr_state=1;
            robot_recr_time = time(NULL);
         } if (recr_state==1) {
            if (robot_recr_time > time(NULL)-5) {
               if (robot_nearest_id<0) {
                  std::cout << DEBUG << "Nobody is listening, Go back to exploration" << std::endl;
                  equids->getJockey(J_POSITION)->stop(true);
                  state = S_EXPLORATION;
               } else {
                  memcpy(&recruiting_accp[0], &equids->getJockey(J_POSITION)->actual_position, sizeof(UbiPosition));
                  recruitingMsg.type = MSG_HELP_ACP;
                  recruitingMsg.data = recruiting_accp;
                  recruitingMsg.len=sizeof(UbiPosition);
                  packedMessage = CMessage::packToZBMessage(robot_nearest_id,
                     recruitingMsg.type, recruitingMsg.data,
                     recruitingMsg.len);
                  equids->getJockey(J_ZBMESSENGER)->SendMessage(packedMessage);
                  recr_state=2;
                  robot_recr_time = time(NULL);
               }
            } 
         } else if (recr_state==2) {
         }
         break;
      }
		case S_QUIT:
			quit = true;
			break;
		}
		usleep(100000);
	}
}

/**
 * Create this message from scratch now. Normally, this is a message that is obtained from the mapping jockey with the
 * right position, subsequently it is updated by the laserscan jockey with the right object information.
 */
void GrandChallenge1Scenario::CreateMsgLaserDetectObject(CMessage &msg) {
	MappedObjectPosition position;
	int len = sizeof(struct MappedObjectPosition);
	if (msg.data != NULL)
		delete[] msg.data;
	msg.data = new uint8_t[len];
	msg.len = len;
	std::cout << DEBUG << "Send recognition message of size " << len << std::endl;
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
}


