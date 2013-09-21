#include <stdlib.h>
#include "CImageClient.h"
#include "CGui.h"
#include "CTimer.h"
#include <signal.h>

#include <CMessageClient.h>

bool status[20];
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

//#define STORE_IMAGES_ANYWAY

int main(int argc,char* argv[])
{
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	std::string ip_address, command_port, image_port;
	if (argc < 4) {
		std::cerr << "Usage: " << argv[0] << " IP_ADDRESS COMMAND_PORT IMAGE_PORT" << std::endl;
		exit(EXIT_FAILURE);
	} else {
		ip_address = std::string(argv[1]);
		command_port = std::string(argv[2]);
		image_port = std::string(argv[3]);
	}
	bool requirements[0]; // none

	CMessageClient cmd_client;
	cmd_client.init(ip_address.c_str(), command_port.c_str(), requirements);


	message.data = new uint8_t[MAX_MESSAGE_SIZE];

	CMessage msg;
	msg.type = MSG_INIT;
	msg.len = 0;
	cmd_client.sendMessage(&msg);

	sleep(1);

	msg.type = MSG_CAM_VIDEOSTREAM_START;
	msg.len = 0;
	cmd_client.sendMessage(&msg);

	sleep(1);

	msg.type = MSG_START;
	msg.len = 0;
	cmd_client.sendMessage(&msg);

	// sleep 2 seconds
	sleep(3);

	CImageClient* client = new CImageClient();
	int error = client->connectServer(ip_address.c_str(),image_port.c_str());
	if (error < 0) {
		std::cerr << "Could not connect to the image server" << std::endl;
		return EXIT_FAILURE;
	}
	CTimer timer;
	//message.type = MSG_GET_IMAGE;
	image = new CRawImage();
	int runs=0;
	while (stop == false){
		//		client->sendMessage(message);
		client->sendSmallMessage(0);
		client->checkForImage(image);	
		//message.type = MSG_GET_IMAGE;
		gui.drawImage(image);

		//fill the status HERE
		for (int i = 0;i<20;i++) status[i] = ((runs/(int)(pow(2,i)))%2)==1;
		gui.drawStatus(status);
#ifdef STORE_IMAGES_ANYWAY
		image->saveBmp();
#endif
		gui.update();
		processKeys(&cmd_client);
		//usleep(100000);
		runs++;
	}
	delete client;
	return EXIT_SUCCESS;
}
