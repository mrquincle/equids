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
#include <CMotorsCalib.h>
#include <Map.h>
#include <Mapping.h>

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

//#define DEBUGODOCALIB
#define NAME "MotorCalibration"
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "
#define UBISENCE_POSITION_CHANNEL 55
#define UBISENCE_MESSAGE_SERVER_CHANNEL 56
#define DEBUGSTRING NAME << '[' << getpid() << "] " << __func__ << "(): "
typedef enum {
	WAIT = 0, CALIBRATION
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

//actions hadlers
CMotorsCalib* motor_calibration = NULL;
Map* slamMap = NULL;
Mapping* mapProcedure = NULL;
bool stop = false;
DetectedBlob* detectedBlob = NULL;

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
	motor->setSpeeds(0, 0);
	usleep(20000);
}

bool isPossible(DetectedBlob* detectedBlob){
	return (detectedBlob->x < 4 && detectedBlob->x > -4 && detectedBlob->y < 4 && detectedBlob->y > -4 && detectedBlob->z < 4 && detectedBlob->z > -4);
}

void readMessages() {
	messagee = message_server->getMessage();
	if (messagee.type != MSG_NONE) {
		//	fprintf(stdout,"Command: %s %i %i %i %i\n",message.getStrType(),message.value1,message.value2,message.value3,message.value4);
		switch (messagee.type) {
		case MSG_INIT: {
			/*
			robot->pauseSPI(false);
			while (robot->isSPIPaused()) {
				usleep(10000);
			}*/
			initialize();
			actualTask = WAIT;
			motor_calibration = new CMotorsCalib(robot, robot_type, motor);
			robot->pauseSPI(true);
			while (!robot->isSPIPaused()) {
				usleep(10000);
			}
			usleep(100000);
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);

		}
			;
			break;
		case MSG_START: {

			if (motor_calibration->readCalibResult()) {
				printf("calibration succesfully readed: %e %e %e\n",
						motor->odometry_koef1, motor->odometry_koef2,
						motor->odometry_koef3);
				MotorCalibResult calibres = { motor->odometry_koef1,
										motor->odometry_koef2, motor->odometry_koef3,
										motor->calibratedSpeed };
				message_server->sendMessage(MSG_MOTOR_CALIBRATION_RESULT, &calibres,
										sizeof(MotorCalibResult));
				actualTask = WAIT;
			} else {
				robot->pauseSPI(false);
				while (robot->isSPIPaused()) {
					usleep(10000);
				}
				actualTask = CALIBRATION;
				if (robot_type == RobotBase::ACTIVEWHEEL) {
					ActiveWheel *bot = (ActiveWheel*) robot;
					printf("changing hinge \n");
					bot->MoveHingeToAngle(8.3);
					motor->getPosition()[5] = 2.996730326; //setting hinge to 171.7°
					usleep(500000);

				}
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
			if(messagee.len==0)printf("NULL \n");
			if (messagee.len != 0) {
				printf("1\n");
				DetectedBlobWSize* detected = new DetectedBlobWSize();
				memcpy(detected,  messagee.data,sizeof(DetectedBlobWSize));
				if(detectedBlob!=NULL){
					delete detectedBlob;
				}
				printf("2\n");
				detectedBlob=NULL;
				if(isPossible(&detected->detectedBlob)){
				detectedBlob = new DetectedBlob();
				detectedBlob->x = detected->detectedBlob.x;

				detectedBlob->y = detected->detectedBlob.y;
				detectedBlob->z = detected->detectedBlob.z;
				detectedBlob->phi = detected->detectedBlob.phi;
				printf("detected blob MC %f %f %f %f \n", detected->detectedBlob.x,detected->detectedBlob.y, detected->detectedBlob.z,detected->detectedBlob.phi);
				}
				printf("3\n");
				} else {
				detectedBlob = NULL;
			}
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

	while (!stop) {
		if(detectedBlob != NULL){delete detectedBlob;}
		detectedBlob = NULL;
		readMessages();
		switch (actualTask) {
		case CALIBRATION: {
			/*if (detectedBlob == NULL) {
				printf("NULL detected object \n");
			}*/
			motor_calibration->calibrate(detectedBlob);
#if defined(DEBUGODOCALIB)
				printf("calib state: %d \n", motor_calibration->calibstate);
				printf("filter iteration: %d \n",motor_calibration->filteriteration	);
#endif
			if (motor_calibration->successful) {
				actualTask = WAIT;
				MotorCalibResult calibres = { motor->odometry_koef1,
						motor->odometry_koef2, motor->odometry_koef3,
						motor->calibratedSpeed };
				message_server->sendMessage(MSG_MOTOR_CALIBRATION_RESULT, &calibres,
						sizeof(MotorCalibResult));
			}
			//usleep(2000000);
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
	delete motor_calibration;
	delete message_server;
	std::cout << "Exit Motor Calibration" << std::endl;
	return 0;
}

