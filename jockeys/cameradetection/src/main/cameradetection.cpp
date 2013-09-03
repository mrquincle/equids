#include <sys/types.h>
#include "IRobot.h"
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "termios.h"
#include <signal.h>
#include <messageDataType.h>
#include <CRawImage.h>
#include <CImageServer.h>
#include <CCamera.h>
#include <CTimer.h>
#include <CCircleDetect.h>
#include <CTransformation.h>
#include <CMessageServer.h>
#include <CMessage.h>

#if MULTI_CONTROLLER==true
#include <action/StateEstimate.h>
#include <action/ActionSelection.h>
#endif

#define NAME "CameraDetection"
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "
#define VIDEO_DEVICE "/dev/video0"
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define TRACKED_CIRC_DIAMETER_MAP 0.19
#define INNER_CIRC_DIAMETER_MAP 0.1
#define TRACKED_CIRC_DIAMETER_DOCK 0.012
#define INNER_CIRC_DIAMETER_DOCK 0.004
#define PI 3.14159265

#define DEBUGSTRING NAME << '[' << getpid() << "] " << __func__ << "(): "
typedef enum {
	DETECT_DOCKING = 0, DETECT_MAPPING, DETECT_STAIR, DETECT_NO_TASK
} ActualCameraUsage;

using namespace std;

ActualCameraUsage actualTask = DETECT_NO_TASK;
//message system
CMessageServer* message_server;
CMessage message;
std::string portMS;

//timer maybe unused
CTimer* timer;

//camera server and image
CRawImage* image;
sem_t imageSem;
CImageServer* image_server;
CCamera* camera;
bool swapIMG = false;
std::string portIS;
bool streamVideo = false;

//cicrcle detector for mapping
CCircleDetect* circle_detector;
CTransformation* circle_trans;
STrackedObject o;
SSegment currentSegment;
SSegment lastSegment;
RobotBase::RobotType robot_type;
//cicrcle detector for docking
CCircleDetect *detectorArray[MAX_DOCKING_PATTERNS];
STrackedObject objectArray[MAX_DOCKING_PATTERNS];
SSegment currentSegmentArray[MAX_DOCKING_PATTERNS];
SSegment lastSegmentArray[MAX_DOCKING_PATTERNS];

bool stop = false;
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(0);
	}
}

void switchActualTask(ActualCameraUsage newTask) {

	//test if not switching to same
	if (actualTask != newTask) {
		/*
		 * delete old allocation
		 */
		switch (actualTask) {
		case DETECT_DOCKING: {
			for (int i = 0; i < MAX_DOCKING_PATTERNS; i++) {
				delete detectorArray[i];
			}
			delete circle_trans;
		}
			break;
		case DETECT_MAPPING: {
			delete circle_detector;
			delete circle_trans;
		}
			break;
		case DETECT_STAIR: {

		}
			break;
		default:
			break;
		}

		/*
		 * alocate for new purposes
		 */
		switch (newTask) {
		case DETECT_DOCKING: {
			circle_trans = new CTransformation(IMAGE_WIDTH, IMAGE_HEIGHT,
					TRACKED_CIRC_DIAMETER_DOCK, false);

			for (int i = 0; i < MAX_DOCKING_PATTERNS; i++)
				detectorArray[i] = new CCircleDetect(IMAGE_WIDTH, IMAGE_HEIGHT,
						INNER_CIRC_DIAMETER_DOCK / TRACKED_CIRC_DIAMETER_DOCK);
		}
			;
			break;
		case DETECT_MAPPING: {
			circle_trans = new CTransformation(IMAGE_WIDTH, IMAGE_HEIGHT,
					TRACKED_CIRC_DIAMETER_MAP, false);
			circle_detector = new CCircleDetect(IMAGE_WIDTH, IMAGE_HEIGHT,
					INNER_CIRC_DIAMETER_MAP / TRACKED_CIRC_DIAMETER_MAP);
		}
			;
			break;
		case DETECT_STAIR: {
			//need implementation
		}
			;
			break;
		default:
			break;
		}

	}

	actualTask = newTask;
}

/*
 * function that handle with messages
 */
void readMessages() {
	//	printf("getting message\n");
	message = message_server->getMessage();
	if (message.type != MSG_NONE) {
		//	fprintf(stdout,"Command: %s %i %i %i %i\n",message.getStrType(),message.value1,message.value2,message.value3,message.value4);
		switch (message.type) {
		case MSG_INIT: {
			printf("message init\n");
			//need robot type for image swap
			/*
			robot_type = RobotBase::Initialize(NAME);
			switch (robot_type) {
			case RobotBase::SCOUTBOT: {
				swapIMG = false;
			}
				break;
			default: {
				printf("swapping ................. ..............\n");
				swapIMG = true;
			}
				break;
			}*/
			swapIMG = false;
#ifdef OLD_CAMERA_INIT
			sem_init(&imageSem, 0, 1);
			camera = new CCamera(&imageSem, swapIMG);
			camera->init(VIDEO_DEVICE, IMAGE_WIDTH, IMAGE_HEIGHT);
			image = new CRawImage(IMAGE_WIDTH, IMAGE_HEIGHT);
#endif
			sem_init(&imageSem, 0, 1);

			int cameraDeviceHandler;
			int imgWidth = IMAGE_WIDTH;
			int imgHeight = IMAGE_HEIGHT;
			int bytes_per_pixel = 3;
			camera = new CCamera();
			camera->Init(VIDEO_DEVICE, cameraDeviceHandler, imgWidth, imgHeight);
			image = new CRawImage(imgWidth, imgHeight, bytes_per_pixel);

			image_server = new CImageServer(&imageSem, image);

			std::cout << "Possible image server on port " << portIS
					<< std::endl;

			std::cout << "Possible image server on port " << portIS
					<< std::endl;

			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_START: {
			printf("Start %s\n", NAME);
			actualTask = DETECT_NO_TASK;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_STOP: {
			printf("Stop %s\n", NAME);
			switchActualTask(DETECT_NO_TASK);
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_QUIT: {
			printf("Quit %s\n", NAME);
			switchActualTask(DETECT_NO_TASK);
			stop = true;
		}
			;
			break;
		case MSG_CAM_DETECT_DOCKING: {
			printf("message MSG_CAM_DETECT_DOCKINGt\n");
			switchActualTask(DETECT_DOCKING);
			std::cout << "Initialize Docking" << std::endl;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_CAM_DETECT_MAPPING: {
			printf("message MSG_CAM_DETECT_MAPPING\n");
			switchActualTask(DETECT_MAPPING);
			std::cout << "Initialize mapping" << std::endl;
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_CAM_DETECT_STAIR: {
			switchActualTask(DETECT_STAIR);
			//need implementation
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;

		case MSG_CAM_VIDEOSTREAM_START: {
			printf("Start video stream on %s\n", NAME);
			if (!streamVideo) {
				streamVideo = true;
				image_server->initServer(portIS.c_str());
			}
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		}
			;
			break;
		case MSG_CAM_VIDEOSTREAM_STOP: {
			printf("Stop video stream on %s\n", NAME);
			if (streamVideo) {
				streamVideo = false;
				image_server->stopServer();
			}
			message_server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
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
	std::cout << DEBUG << "Started with params " << argv[1] << ", " << argv[2]
			<< std::endl;

	if (argc > 2) {
		portMS = std::string(argv[1]);
		portIS = std::string(argv[2]);
	} else {
		std::cout << DEBUG
				<< "Usage: message_server_port_number image_server_port_number "
				<< std::endl;
		return 1;
	}

	std::cout << "Create (receiving) message server on port " << portMS
			<< std::endl;

	message_server = new CMessageServer();
	std::cout << "Initialize CMessageServer" << std::endl;
	message_server->initServer(portMS.c_str());

	while (!stop) {
		if (actualTask != DETECT_NO_TASK || streamVideo) {
			//camera->renewImage(image,true);
			camera->renewImage(image, true);
		}
		readMessages();
		switch (actualTask) {
		case DETECT_MAPPING: {
			lastSegment = currentSegment;
			currentSegment = circle_detector->findSegment(image, lastSegment);
			if (currentSegment.valid) {
				o = circle_trans->transform(currentSegment, false);
				int sign = (o.roll > 0) ? -1 : 1;
					DetectedBlob blob = { o.x, o.y, o.z, o.pitch * PI / 180 * sign };
				DetectedBlobWSize blobWSize = { 1, { o.x, o.y, o.z, o.pitch * PI
						/ 180 * sign } };
				printf("%f %f %f %f\n",blob.x,blob.y,blob.z,blob.phi);
				//		std::cout << "MSG_CAM_DETECTED_BLOB_SIZE " << sizeof(DetectedBlobWSize) << std::endl;
				message_server->sendMessage(MSG_CAM_DETECTED_BLOB, &blobWSize,
						sizeof(DetectedBlobWSize));
			} else {
				printf("NULL blob\n");
				message_server->sendMessage(MSG_CAM_DETECTED_BLOB, NULL, 0);
			}
		}
			break;
		case DETECT_DOCKING: {
			DetectedBlob blobArray[MAX_DOCKING_PATTERNS];
			int pocet = 0;
			for (int i = 0; i < MAX_DOCKING_PATTERNS; i++) {
				lastSegmentArray[i] = currentSegmentArray[i];
				currentSegmentArray[i] = detectorArray[i]->findSegment(image,
						lastSegmentArray[i]);

				if (currentSegmentArray[i].valid) {
					objectArray[i] = circle_trans->transform(
							currentSegmentArray[i], false);
					blobArray[pocet].x = objectArray[i].x;
					blobArray[pocet].y = objectArray[i].y;
					blobArray[pocet].z = objectArray[i].z;
					blobArray[pocet].phi =
							(objectArray[i].roll > 0) ?
									-objectArray[i].pitch * PI / 180.0 :
									objectArray[i].pitch * PI / 180.0;
					pocet++;
				}
			}
			if (pocet != 0) {
				DetectedBlobWSizeArray blobArrayWSize;
				blobArrayWSize.size = pocet;
				for (int var = 0; var < pocet; ++var) {
					blobArrayWSize.detectedBlobArray[var] = blobArray[var];
				}
				message_server->sendMessage(MSG_CAM_DETECTED_BLOB_ARRAY,
						&blobArrayWSize, sizeof(DetectedBlobWSizeArray));
			} else {
				message_server->sendMessage(MSG_CAM_DETECTED_BLOB_ARRAY, NULL,
						0);
			}
		}
			break;
		case DETECT_STAIR: {

		}
			break;
		default: {
			//		printf("No actual Task\n");
			usleep(100000);
		}
			;
			break;
		}
	}
	std::cout << "Stopping camera detection jockey" << std::endl;
	return 0;
}

