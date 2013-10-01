#include <stdlib.h>
#include "CImageClient.h"
#include "CGui.h"
#include "CTimer.h"
#include <signal.h>
#include <vector>
#include <ZigBee.h>
#include <iostream>
#include <fstream>
#include <CMessageClient.h>


int i = 0;
int numSaved = 0;
bool stop = false;
CGui gui;
SDL_Event event;
CMessage message;
const int MAX_MESSAGE_SIZE=32;
CRawImage *image;
Uint8 lastKeys[1000];
Uint8* keys;
int keyNumber = 1000;

ZigBee *zigbee = NULL;

struct MotorCommand {
	int16_t forward;
	int16_t radius;
} __attribute__((packed));

MotorCommand motorCommand;

void processKeys(CMessageClient *cmd_client)
{
	SDL_PumpEvents();
	//	while (SDL_PollEvent(&event)){
	//		if (event.type == SDL_MOUSEBUTTONDOWN){
	//			//	message.type = MSG_LEARN;
	////			message.value1 = event.motion.x;
	////			message.value2 = event.motion.y;
	//		}
	//	}
	keys = SDL_GetKeyState(&keyNumber);
	//if (keys[SDLK_r]) message.type = MSG_RESET;
	if (keys[SDLK_ESCAPE]) {
		stop = true;
		message.type = MSG_CAM_VIDEOSTREAM_STOP;
		message.len = 0;
		cmd_client->sendMessage(&message);
	}
	if (keys[SDLK_q]) {
		stop = true;
		message.type = MSG_CAM_VIDEOSTREAM_STOP;
		message.len = 0;
		cmd_client->sendMessage(&message);
		sleep(1);
		message.type = MSG_QUIT;
		cmd_client->sendMessage(&message);
		message.len = 0;
		sleep(1);
	}
	//if (keys[SDLK_p]) message.type = MSG_PAUSE;
	//if (keys[SDLK_o]) message.type = MSG_RESUME;
	//if (keys[SDLK_l]) message.type = MSG_STEP;
	//if (keys[SDLK_s]) message.type = MSG_SAVE;
	if (keys[SDLK_w]) {
		message.type = MSG_SPEED;
		motorCommand.forward = 40;
		motorCommand.radius = 1000;
		memcpy(message.data, &motorCommand, sizeof(MotorCommand));
		message.len = sizeof(MotorCommand);
		cmd_client->sendMessage(&message);
	}
	if (keys[SDLK_s]) {
		message.type = MSG_SPEED;
		motorCommand.forward = -40;
		motorCommand.radius = 1000;
		memcpy(message.data, &motorCommand, sizeof(MotorCommand));
		message.len = sizeof(MotorCommand);
		cmd_client->sendMessage(&message);
	}
	if (keys[SDLK_a]) {
		message.type = MSG_SPEED;
		motorCommand.forward = 40;
		motorCommand.radius = 20;
		memcpy(message.data, &motorCommand, sizeof(MotorCommand));
		message.len = sizeof(MotorCommand);
		cmd_client->sendMessage(&message);
	}
	if (keys[SDLK_d]) {
		message.type = MSG_SPEED;
		motorCommand.forward = 40;
		motorCommand.radius = -20;
		memcpy(message.data, &motorCommand, sizeof(MotorCommand));
		message.len = sizeof(MotorCommand);
		cmd_client->sendMessage(&message);
	}


	/*	if (keys[SDLK_m]) recognition->increaseTolerance();
	if (keys[SDLK_n])  recognition->decreaseTolerance();
	if (keys[SDLK_l]) recognition->loadColorMap();
	if (keys[SDLK_s]) recognition->saveColorMap();*/
	if (keys[SDLK_RETURN])image->saveBmp();
	memcpy(lastKeys,keys,keyNumber);
}

void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(0);
	}
}

//! Quick and dirty to get id from ip_address
void get_id(std::string ip_address, std::vector<int> & result) {
	std::stringstream ss(ip_address);
	result.clear();
	while( ss.good() ) {
		std::string substr;
		getline( ss, substr, '.' );
		int n = atoi(substr.c_str());
		result.push_back( n );
	}
}

void write_jockeys(std::vector<vocab_t> & jockey_ids) {
	std::cout << "Write all jockeys to jockeys file. " << std::endl;
	std::cout << "Do run script src/createpics.sh and create a symbolic link from bin/images" << std::endl;
	std::ofstream f;
	f.open("jockeys");
	f << "actionselection" << std::endl;
	int id;
	id = V_ACTIONSELECTION;
	jockey_ids.push_back(id);
	f << "avoidir" << std::endl;
	id = V_AVOIDIR;
	jockey_ids.push_back(id);
	f << "cameradetection" << std::endl;
	id = V_CAMERADETECTION;
	jockey_ids.push_back(id);
	f << "docking" << std::endl;
	id = V_DOCKING;
	jockey_ids.push_back(id);
	f << "mapping" << std::endl;
	id = V_MAPPING;
	jockey_ids.push_back(id);
	f << "laserscan" << std::endl;
	id = V_LASERSCAN;
	jockey_ids.push_back(id);
	f << "ubiposition" << std::endl;
	id = V_UBIPOSITION;
	jockey_ids.push_back(id);
	f << "zigbeemsg" << std::endl;
	id = V_ZIGBEEMSG;
	jockey_ids.push_back(id);
}

//#define STORE_IMAGES_ANYWAY

int main(int argc,char* argv[])
{
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	std::string ip_address, command_port, image_port;
	if (argc < 4) {
		std::cerr << "Usage: " << argv[0] << " IP_ADDRESS COMMAND_PORT IMAGE_PORT [ZIGBEE]" << std::endl;
		exit(EXIT_FAILURE);
	} else {
		ip_address = std::string(argv[1]);
		command_port = std::string(argv[2]);
		image_port = std::string(argv[3]);
	}
	assert(ip_address != "");

	std::vector<int> ip_address_int;
	get_id(ip_address, ip_address_int);
	assert (ip_address_int.size() == 4); // ip address should have 4 fields
	int id = ip_address_int[3];
	std::cout << "Connect to robot with id " << id << std::endl;

	bool enable_zigbee = false; bool enable_control = false; bool enable_camera = false;
	if (argc == 5) {
		std::string arg5 = std::string(argv[4]);
		if (arg5.find("zigbee") != std::string::npos) {
			enable_zigbee = true;
		}
		if (arg5.find("control") != std::string::npos) {
			enable_control = true;
		}
		if (arg5.find("camera") != std::string::npos) {
			enable_camera = true;
		}
	}

	bool requirements[0]; // none

	if (enable_zigbee) {
		zigbee = new ZigBee();
		zigbee->Init();
	}

	CMessageClient cmd_client;
	if (enable_control) {
		std::cout << "Enable control" << std::endl;
		cmd_client.init(ip_address.c_str(), command_port.c_str(), requirements);

		std::cout << "Use the keys [a,s,d,w] to move the robot" << std::endl;

		message.data = new uint8_t[MAX_MESSAGE_SIZE];

		CMessage msg;
		msg.type = MSG_INIT;
		msg.len = 0;
		cmd_client.sendMessage(&msg);

		sleep(1);

		std::cout << "Send MSG_CAM_VIDEOSTREAM_START" << std::endl;
		msg.type = MSG_CAM_VIDEOSTREAM_START;
		msg.len = 0;
		cmd_client.sendMessage(&msg);

		sleep(1);

		msg.type = MSG_START;
		msg.len = 0;
		cmd_client.sendMessage(&msg);

		// sleep 2 seconds
		sleep(3);
	}

	CImageClient* client = new CImageClient();

	std::cout << "Quick test, this should be MSG_ACTIVE_JOCKEYS: \"" << StrMessage[MSG_ACTIVE_JOCKEYS] << "\" to get active jockeys list" << std::endl;

	std::vector<vocab_t> jockey_ids;
	write_jockeys(jockey_ids);
	int jockey_count = jockey_ids.size();
	gui.initJockeys(jockey_count);
	bool status[jockey_count];
	for (int i = 0; i < jockey_ids.size(); i++) {
		status[i] = false;
	}

	CTimer timer;
	message.type = MSG_NONE;
	message.data = NULL;
	message.len = 0;
	image = new CRawImage();
	int runs=0;

	int zigb_request_count = 10;
	int zigb_request = zigb_request_count;

	int cam_request_count = 20;
	int cam_request = cam_request_count;

	bool connected = false;
	while (stop == false) {
		if (enable_camera) {
			if (!connected) {
				if (!--cam_request) {
					std::cout << "Connect to image port on the image server: " << image_port.c_str() << std::endl;
					int error = client->connectServer(ip_address.c_str(),image_port.c_str());
					if (error < 0) {
						std::cerr << "Could not connect to image port on the image server, try next time" << std::endl;
						cam_request = cam_request_count;
					} else {
						connected = true;
					}
				}
			}
		}

		message = zigbee->readMessage();
		if (message.type == MSG_ACTIVE_JOCKEYS) {
			std::cout << "Got message back about active jockeys" << std::endl;
			if (message.fromRobot != id) {
				std::cout << "Message is from another robot" << std::endl;
			} else {
				std::cout << "Message is for us" << std::endl;
				int len = message.len;
				if (len == 0) {
					std::cout << "No jockeys active" << std::endl;
				} else {
					std::cout << "Message length = " << len << std::endl;
					int int_len = ((len * sizeof(char)) / sizeof(int));
					std::cout << "Number of jockeys active: " << int_len << std::endl;
					int *jockey_arr = (int*)message.data;
					for (int i = 0; i < jockey_ids.size(); i++) {
						status[i] = false;
					}
					for (int a = 0; a < int_len; a++) {
						for (int i = 0; i < jockey_ids.size(); i++) {
							if (jockey_ids[i] == jockey_arr[a]) {
								std::cout << "Jockeys with id " << a << " and index " << i << " active" << std::endl;
								status[i] = true;
							}
						}
					}
				}
				std::cout << "Update gui status" << std::endl;
				gui.drawStatus(status);
			}
		}

		if (connected) {
			//		client->sendMessage(message);
			client->sendSmallMessage(0);
			int result = client->checkForImage(image);
			if (result) {
				//message.type = MSG_GET_IMAGE;
				gui.drawImage(image);
			}
		}

		if (!--zigb_request) {
			//fill the status HERE
			message.type = MSG_ACTIVE_JOCKEYS;
			message.data = NULL;
			message.len = 0;
			std::cout << "Send message \"" << StrMessage[message.type] << "\" to get active jockeys list" << std::endl;
			bool success = zigbee->zigbeeSend(message, id);
			if (success) {
				zigb_request = zigb_request_count;
			} else {
				std::cout << "Zigbee request not successfully sent" << std::endl;
				client->disconnectServer();
				connected = false;
			}
		}


		//		for (int i = 0;i<jockey_count;i++) status[i] = ((runs/(int)(pow(2,i)))%2)==1;
		//		message.type = MSG_
		//		client->sendMessage();

#ifdef STORE_IMAGES_ANYWAY
		image->saveBmp();
#endif
		gui.update();
		if (enable_control) {
			processKeys(&cmd_client);
		}
		usleep(500000);
		runs++;
	}
	delete client;
	return EXIT_SUCCESS;
}
