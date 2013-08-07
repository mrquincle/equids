#ifndef __CROBOT_H__
#define __CROBOT_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "CRecognition.h"
#include "cmath.h"

//prikaz pro robot udavajici jak rychle se ma zatacet a jak rychle jet vpred

typedef struct{
	int forward;
	int turn;	
}SRobotCommand;

class CRobot
{
	public:
		CRobot();
		~CRobot();
		int getOdometry();
		void sendCommand(SRobotCommand command);
		void stop();
		bool init(const char* port);
		SRobotCommand computeCommand(SPixelPosition position);

	protected:
		int lastOdometry;
		unsigned char buffer[10];
		int port;
		bool initialized;
		int commDelay;
};

#endif
