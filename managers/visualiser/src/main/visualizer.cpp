#include <stdlib.h>
#include "CImageClient.h"
#include "CGui.h"
#include "CTimer.h"
 #include <signal.h>

int i = 0;
int numSaved = 0;
bool stop = false;
CGui gui;
SDL_Event event;
CMessage message;
CRawImage *image;
Uint8 lastKeys[1000];
Uint8* keys;
int keyNumber = 1000;

void processKeys()
{
	while (SDL_PollEvent(&event)){
		if (event.type == SDL_MOUSEBUTTONDOWN){
		       //	message.type = MSG_LEARN;
		       	message.value1 = event.motion.x;
			message.value2 = event.motion.y;
		}
	}
	keys = SDL_GetKeyState(&keyNumber);
	//if (keys[SDLK_r]) message.type = MSG_RESET;
	if (keys[SDLK_ESCAPE]) stop = true;
	//if (keys[SDLK_p]) message.type = MSG_PAUSE;
	//if (keys[SDLK_o]) message.type = MSG_RESUME;
	//if (keys[SDLK_l]) message.type = MSG_STEP;
	//if (keys[SDLK_s]) message.type = MSG_SAVE;
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

int main(int argc,char* argv[])
{
	struct sigaction a;
		a.sa_handler = &interrupt_signal_handler;
		sigaction(SIGINT, &a, NULL);

	if (argc < 3) {
		fprintf(stderr, "Add ip address of image server and port as argument\n");
		return EXIT_FAILURE;
	}
	CImageClient* client = new CImageClient();
//	client->connectServer("127.0.0.1","10000");
	client->connectServer(argv[1],argv[2]);
	CTimer timer;
	//message.type = MSG_GET_IMAGE;
	image = new CRawImage();
	while (stop == false){
//		client->sendMessage(message);
		client->sendSmallMessage(0);
		client->checkForImage(image);	
		//message.type = MSG_GET_IMAGE;
		gui.drawImage(image);
		image->saveBmp();
		gui.update();
		processKeys();
		//usleep(100000);
	}
	delete client;
	return 0;
}
