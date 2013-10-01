/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file LaserScanController.cpp
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
 * @date      Sep 2, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#include <LaserScanController.h>
#include <CTextLog.h>

#include <syslog.h> // LOG_DEBUG

//! The name of the controller can be used for controller selection
static const std::string NAME = "LaserScan";

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

LaserScanController::LaserScanController(): scan(NULL), imageSem(new sem_t()), image_server(NULL), images(), patch(),
		mosaic_image(NULL), streaming(false), motors(NULL), create_mosaic(true), initialized_periphery(false) {
	images.resize(4);
	log_level = LOG_INFO;
	exclusive_camera = false;
	semaphore_set = false;
	calc_distance = true;
}

LaserScanController::~LaserScanController() {
	// flush, because deallocation can go wrong somewhere and we'd have a memory dump
	std::cout << std::endl << std::flush;
	std::cout << DEBUG << "Robot object is automatically deleted by the factory." << std::endl;

	std::cout << DEBUG << "Delete CLaserScan instance" << std::endl;
	delete scan;
	std::cout << DEBUG << "Delete CMotors instance" << std::endl;
	delete motors;
}

void LaserScanController::initRobotPeriphery() {
	if (initialized_periphery) return;

	robot->SetLEDAll(0, LED_OFF);
	robot->SetLEDAll(1, LED_RED);
	robot->SetLEDAll(2, LED_GREEN);

	std::ostringstream msg;
	msg.clear(); msg.str(""); msg << NAME << '[' << getpid() << "] ";

	std::cout << DEBUG << "Setup laser functionality" << std::endl;
	scan = new CLaserScan(robot, robot_type, 640, 480, 640);
	scan->setLogPrefix(msg.str());
	scan->Init();

	motors = new CMotors(robot, robot_type);
	motors->setLogPrefix(msg.str());
	motors->init();

	// use an environmental variable STREAM_ONLY_CAMERA, by default flip camera it is false
	create_mosaic = true;
	char *str_stream_only_camera = getenv("STREAM_ONLY_CAMERA");
	if (str_stream_only_camera) {
		std::string s = std::string(str_stream_only_camera);
		std::transform(s.begin(), s.end(), s.begin(), ::tolower);
		if (s == "true") {
			create_mosaic = false;
		}
	}

	initialized_periphery = true;
}

bool LaserScanController::initialized() {
	if (!initialized_robot || !initialized_periphery) {
		std::cerr << DEBUG << "Robot not initialized!" << std::endl;
		return false;
	}
	return true;
}

void LaserScanController::head_back(int factor) {
	if (log_level >= LOG_INFO) {
		std::cout << DEBUG << "Go back for a few seconds" << std::endl;
	}
#ifdef USE_CVUT
	int speed = 30; int turn = 0;
	motors->setSpeeds(-speed, turn);
#else
	int speed = 30; int radius = 1000;
	motors->setRadianSpeeds(-speed, radius);
#endif
	sleep(factor);
	motors->set_to_zero();
}

void LaserScanController::setSemaphore(sem_t *cap_sem) {
	capture_sem = cap_sem;
	semaphore_set = true;
}

void LaserScanController::motorCommand(MotorCommand &motorCommand) {
	if (!initialized()) return;

	if (motors == NULL) {
		std::cerr << DEBUG << "Motor is null, did you call initRobotPeriphery through sending MSG_INIT!?" << std::endl;
		return;
	}
	motors->setRadianSpeeds(motorCommand.forward, motorCommand.radius);
	usleep(100000);
}

void LaserScanController::printDetectedObject(ObjectType object) {
	std::cout << DEBUG << "Detected object" << std::endl;
	switch (object) {
	case O_SMALL_STEP: case O_LARGE_STEP:
		std::cout << step << std::endl;

		// set also leds
		robot->SetLEDAll(1, LED_ORANGE);
		robot->SetLEDAll(2, LED_ORANGE);
		robot->SetLEDAll(3, LED_ORANGE);
		break;
		//	case O_SMALL_STEP: std::cout << small << std::endl << step << std::endl; break;
		//case O_LARGE_STEP: std::cout << large << std::endl << step << std::endl; break;
	case O_WALL:
		std::cout << wall << std::endl;

		// set also leds
		robot->SetLEDAll(1, LED_GREEN);
		robot->SetLEDAll(2, LED_GREEN);
		robot->SetLEDAll(3, LED_GREEN);
		break;
	}
}

ObjectType LaserScanController::getDetectedObject() {
	if (!initialized()) return O_NOTHING;

	std::cout << DEBUG << "Detect object" << std::endl;

	// overwrite position.type
	ObjectType object;
	int distance;

	scan->GetRecognizedObject(object, distance);

	printDetectedObject(object);

	return object;
}

bool LaserScanController::getDistance(int &distance) {
	scan->GetDistance(distance);
	if (distance == -1) return false;
	if (distance == 255) return false;
	return true;
}


void LaserScanController::sendDetectedObject(const ObjectType object, MappedObjectPosition &obj_position) {
	if (!initialized()) return;

	std::cout << DEBUG << "Send detected object" << std::endl;
	printDetectedObject(object);
//
//	// overwrite position.type
//	ObjectType object;
//	int distance;
//
//	scan->GetRecognizedObject(object, distance);

	switch(object) {
	case O_WALL:
		std::cout << DEBUG << "Send wall" << std::endl;
		obj_position.type = WALL;
		break;
	case O_SMALL_STEP:
		std::cout << DEBUG << "Send step" << std::endl;
		obj_position.type = SMALL_STEP;
		break;
	case O_LARGE_STEP:
		std::cout << DEBUG << "Send step" << std::endl;
		obj_position.type = LARGE_STEP;
		break;
	default:
		std::cout << DEBUG << "Send unidentified" << std::endl;
		obj_position.type = UNIDENTIFIED;
		break;
	}

	// overwrite message type
	CMessage msg;
	msg.type = MSG_MAP_DATA;

	// overwrite sender id
	obj_position.mappedBy = robot_id;

	// set relative position
	// assuming that phi is from -pi to +pi, and 0 at [x,y]=[+1,0].
//	obj_position.xPosition += std::sin(obj_position.phiPosition) * distance;
//	obj_position.yPosition += std::cos(obj_position.phiPosition) * distance;

	// set payload
	msg.len = sizeof(struct MappedObjectPosition);
	msg.data = new uint8_t[msg.len];
	memcpy(msg.data, &obj_position, msg.len);

	// send message
	server->sendMessage(msg);

	// delete payload of message
	if (msg.data != NULL) {
		delete [] msg.data;
	}
	std::cout << DEBUG << "Detection message of " << StrMapObjectType[obj_position.type] << std::endl;
}

/**
 * Just prints distance to an object or anything.
 */
void LaserScanController::tick() {
	if (!initialized()) return;

	int distance = 0;

	if (streaming) {
		if (semaphore_set) {
			std::cout << DEBUG << "Wait for semaphore (from e.g. streaming thread)" << std::endl;
			sem_wait(capture_sem);
		}
	}

#ifdef TEST
	ObjectType object;
	scan->GetRecognizedObject(object, distance);
	printDetectedObject(object);
#else
	if (calc_distance) {
		scan->GetDistance(distance);
	}
#endif

	if (calc_distance) {
		if (distance == 255) {
			std::cout << DEBUG << "Nothing seen at maximum range (distance more than 50 cm)" << std::endl;
		} else if (distance == -1) {
			std::cout << DEBUG << "Too noisy to define distance" << std::endl;
		} else if (distance > 0) {
			std::cout << DEBUG << "Distance: " << distance << " cm" << std::endl;
		}
	}

	if (streaming) {

		if (create_mosaic) {
			assert(mosaic_image != NULL);

			if (log_level >= LOG_DEBUG) std::cout << DEBUG << "Compress images so they fit one mosaic image" << std::endl;
			// fill for patches
			for (int i = 0; i < 4; i++) {
				images[i]->compress(patch[i]);
			}

			mosaic_image->setPatch(0, 0, patch[0]);
			mosaic_image->setPatch(0, 1, patch[1]);
			mosaic_image->setPatch(1, 0, patch[2]);
			mosaic_image->setPatch(1, 1, patch[3]);

			if (log_level >= LOG_DEBUG) std::cout << DEBUG << "Written all subimages to one image" << std::endl;

		} else {
			// pick one of the images
			mosaic_image = images[2];
		}

		if (sem_post(imageSem) == -1) {
			std::cerr << DEBUG << "Fail to sem_post image semaphore" << std::endl;
		} else {
			//			if (log_level >= LOG_DEBUG)
			int value;
			sem_getvalue(imageSem, &value);
			std::cout << DEBUG << "Signaled CImageServer through incrementing semaphore to " << value << std::endl;
		}
	}

	// every 0.1 seconds
	usleep(100000);
}

void LaserScanController::pause() {
	if (motors != NULL) {
		if (log_level >= LOG_DEBUG) std::cout << DEBUG << "Halt, stop motors" << std::endl;
		motors->set_to_zero();
		//		motors->halt();
		sleep(1);
	}
	// do not start yet
	if (!exclusive_camera) scan->Pause();

	CController::pause();
}

void LaserScanController::start() {
	if (log_level >= LOG_DEBUG) std::cout << DEBUG << "Start motors" << std::endl;
	if (!scan->isStarted()) {
		scan->Start();
	}
	CController::start();
}

void LaserScanController::startVideoStream(std::string port) {
	if (log_level >= LOG_DEBUG) printf("%s(): configure streaming of images...\n", __func__);

	assert (scan != NULL);
	images[0] = scan->getRedDiffImg();
	images[1] = scan->getRGBDiffImg();
	images[2] = scan->getImg1();
	images[3] = scan->getImg2();

	if (create_mosaic) {
		mosaic_image = new CRawImage(640,480,3);
	} else {
		mosaic_image = images[2];
	}

	for (int i = 0; i < 4; i++)
		assert (images[i] != NULL);

	if (create_mosaic) {
		if (log_level >= LOG_DEBUG) printf("%s(): create mosaic image...\n", __func__);
		//			if (log_level >= LOG_DEBUG) printf("%s(): initialize patches...\n", __func__);
		for (int i = 0; i < 4; i++) {
			patch[i].init(640/2, 480/2);
		}
	}

	sem_init(imageSem, 0, 0); // do not send first image, only at sem_post, see below

	image_server = new CImageServer(imageSem, mosaic_image);
	image_server->initServer(port.c_str());

	// does not always work, so just disable for now
//	setSemaphore(&image_server->captureSem);
	//if (log_level >= LOG_DEBUG) printf("%s(): create semaphore for streaming images at the right moment...\n", __func__);

	streaming = true;
}

void LaserScanController::stopVideoStream() {
	if (mosaic_image != NULL)
		delete mosaic_image;

	if (create_mosaic) {
		for (int i = 0; i < 4; i++) {
			patch[i].free();
		}
	}

	if (log_level >= LOG_DEBUG) std::cout << DEBUG << "Stop image server " << std::endl;

	image_server->stopServer();

	streaming = false;
}

void LaserScanController::testCamera() {
	int cameraDeviceHandler;
	int imgWidth = 640;
	int imgHeight = 480;
	int bpp = 3;
	CCamera camera;
	camera.Init(imgWidth, imgHeight);
	camera.Start("/dev/video0", cameraDeviceHandler);
	CRawImage *image = new CRawImage(imgWidth, imgHeight, bpp);
	camera.renewImage(image, true);

	image->plotCenter();
	//	image->plotLine(30,30);
	image->saveBmp("test_camera.bmp");
	camera.Stop();
	sleep(1);
}
