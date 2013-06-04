/**
 * Logging file
 */
#include <IRobot.h>
#include "IRobotWrapper.h"
#include "IRobotBaseWrapper.h"

#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <signal.h>

#include <CLaser.h>

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

//#define MAINFILE

std::string NAME = "LaserTest";

void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(0);
	}
}

// on

int main(int argc, char **argv) {
	int nof_switches = 10;

//	RobotBase::RobotType robot_type = RobotBase::Initialize(NAME);

	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	IRobotFactory factory;
	RobotBase* robot = factory.GetRobot();
	RobotBase::RobotType robot_type = factory.GetType();

//	RobotBase* robot = RobotBase::Instance();

	switch(robot_type) {
	case RobotBase::UNKNOWN: std::cout << "Detected unknown robot" << std::endl; break;
	case RobotBase::KABOT: std::cout << "Detected Karlsruhe robot" << std::endl; break;
	case RobotBase::ACTIVEWHEEL: std::cout << "Detected Active Wheel robot" << std::endl; break;
	case RobotBase::SCOUTBOT: std::cout << "Detected Scout robot" << std::endl; break;
	default:
		std::cout << "No known type (even not unknown). Did initialization go well?" << std::endl;
    }

//	RobotBase::RobotType type = RobotBase::Initialize("LoggingServer");	
//	RobotBase* bot = RobotBase::Instance();

//	printf("Start drive and sensor functionality\n");
//	CSensors sensors(bot, type);
//	sensors.Init("/dev/ttyBF0", false);

/*
	std::cout << "Test laser and leds directly" << std::endl;
	ScoutBot & rob = *(ScoutBot*)robot;
	for (int i = 0; i < 10; ++i) {
		int k = i%4;
		std::cout << "Print side " << k << std::endl;
		rob.right.laser.on();
		rob.sides[k]->led.all(LED_MAGENTA);
		rob.sides[k]->led.single((LEDPosition)(k), LED_MAGENTA);
		sleep(1);
		rob.right.laser.off();
		rob.sides[k]->led.all(LED_WHITE);
		rob.sides[k]->led.single((LEDPosition)(k), LED_WHITE);
		sleep(1);
	}
*/

	std::cout << "Setup laser functionality" << std::endl;
	CLaser laser(robot, robot_type);
	for (int i = 0; i < nof_switches; ++i) {
		(i % 2) ? laser.Off() : laser.On();
		sleep(2);
	}

	fprintf(stdout,"Stopping laser test.");
	return 0;
}


