/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Get a laser scan from the combination of laser and camera
 * @file laserscan.cpp
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
 * @date      Jul 30, 2012
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

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
#include <iostream> //flush
#include <cassert>

/***********************************************************************************************************************
 * Middleware includes
 **********************************************************************************************************************/

#include <IRobot.h>

/***********************************************************************************************************************
 * Jockey framework includes
 **********************************************************************************************************************/

#include <CLaserScan.h>
//#include <CCamera.h>
#include <CImageServer.h>

#include <CMessageServer.h>

/***********************************************************************************************************************
 * Most important configuration parameters
 **********************************************************************************************************************/

#define STREAM_IMAGES

static const int NUMBER_OF_TICKS = 100;

//! Use verbosity level from SpiStream.h
verbose_level_t verbosity = QUIET;

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "LaserScan";

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		//RobotBase::MSPReset();
		exit(0);
	}
}

void safe_close() {
	// flush, because deallocation can go wrong somewhere and we'd have a memory dump
	std::cout << std::endl << std::flush;
	printf("Robot object is automatically deleted by the factory.\n");
}

RobotBase *start(RobotBase::RobotType &type) {

	std::cout << "Run " << NAME << " compiled at time " << __TIME__ << std::endl;

	type = RobotBase::Initialize(NAME);
	RobotBase* robot = RobotBase::Instance();
	for (int i = 0; i < 4; ++i)
		robot->SetPrintEnabled(i, false);
	return robot;
}

#ifdef JUST_TEST_CAMERA
void test_camera() {
	int cameraDeviceHandler;
	int imgWidth = 640;
	int imgHeight = 480;
	int bpp = 3;
	CCamera camera;
	camera.Init("/dev/video0", cameraDeviceHandler, imgWidth, imgHeight);
	CRawImage *image = new CRawImage(imgWidth, imgHeight, bpp);
	camera.renewImage(image, true);

	image->plotCenter();
	//	image->plotLine(30,30);
	image->saveBmp("test_camera.bmp");
	sleep(1);
	safe_close();
}
#endif

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	sem_t imageSem;

	std::string port;
	if (argc == 2) {
		// first parameter must be the port for the action selection to reach this jockey
		port = std::string(argv[1]);
		std::cout << "Started with OK params " << argv[1] << std::endl;
	} else {
		std::cout << "Usage: [port_number]" << std::endl;
		return EXIT_FAILURE;
	}


	//	IRobotFactory factory;
	//	RobotBase* robot = factory.GetRobot();
	//	RobotBase::RobotType robot_type = factory.GetType();

	// old irobot
	RobotBase::RobotType robot_type;
	RobotBase* robot = start(robot_type);

	robot->SetLEDAll(0, LED_OFF);
	robot->SetLEDAll(1, LED_RED);
	robot->SetLEDAll(2, LED_GREEN);

#ifdef JUST_TEST_CAMERA
	test_camera();
	return 0;
#endif

	std::cout << "Setup laser functionality" << std::endl;
	CLaserScan scan(robot, robot_type, 640, 480, 640);
	scan.Init();

#ifdef STREAM_IMAGES
	if (verbosity >= DEBUGALL) printf("%s(): configure streaming of images...\n", __func__);

	CImageServer* image_server;
	CRawImage *images[4];
	images[0] = scan.getRedDiffImg();
	images[1] = scan.getRGBDiffImg();
	images[2] = scan.getImg1();
	images[3] = scan.getImg2();

	for (int i = 0; i < 4; i++)
		assert (images[i] != NULL);

	Patch p[4];
	if (verbosity >= DEBUGALL) printf("%s(): initialize patches...\n", __func__);
	for (int i = 0; i < 4; i++) {
		p[i].init(640/2, 480/2);
	}

	if (verbosity >= DEBUGALL) printf("%s(): create mosaic image...\n", __func__);
	CRawImage mosaic_image(640,480,3);

	image_server = new CImageServer(&imageSem, &mosaic_image);
	//	image_server = new CImageServer(&imageSem, images[2]);
	image_server->initServer("10002");
	//sem_init(&imageSem, 0, 1);
	if (verbosity >= DEBUGALL) printf("%s(): create semaphore for streaming images at the right moment...\n", __func__);
	sem_init(&imageSem, 0, 0); // do not send first image, only at sem_post, see below

#endif

	CMessageServer *server;
	server = new CMessageServer();
	server->initServer(port.c_str());
	CMessage message;
	bool stopController = false;

	int ticks = NUMBER_OF_TICKS;
	for (int t = 0; t < ticks && !stopController; ++t) {

		message = server->getMessage();
		if (message.type != MSG_NONE)
			std::cout << "Command: " << message.getStrType() << ' ' << message.value1 << ',' \
			<< message.value2 << std::endl;

		switch (message.type) {
		case MSG_START: {
			robot->pauseSPI(false);
			std::cout << argv[0] << ": started controller now and started SPI communication" << std::endl;
			server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
			break;
		}
		case MSG_STOP: {
			robot->pauseSPI(true);
			std::cout << argv[0] << ": paused controller, and pause SPI" << std::endl;
			server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
			break;
		}
		case MSG_QUIT: {
			std::cout << argv[0] << ": quit controller" << std::endl;
			robot->pauseSPI(true);
			stopController = true;
			break;
		}
		case MSG_NONE: {
			// always sleep
			usleep(10000);
			break;
		}
		default: {
			std::cerr << argv[0] << ": did not understand message " << message.type << std::endl;
			break;
		}
		}

		int distance = 0;
		scan.GetDistance(distance);
		std::cout << "Distance: " << distance << " cm" << std::endl;

		if (verbosity >= DEBUGALL) std::cout << "Compress images so they fit one mosaic image" << std::endl;
		// fill for patches
		for (int i = 0; i < 4; i++) {
			images[i]->compress(p[i]);
		}

		mosaic_image.setPatch(0, 0, p[0]);
		mosaic_image.setPatch(0, 1, p[1]);
		mosaic_image.setPatch(1, 0, p[2]);
		mosaic_image.setPatch(1, 1, p[3]);

		if (sem_post(&imageSem) == -1) {
			std::cerr << "Fail to sem_post image semaphore" << std::endl;
		}
		if (verbosity >= DEBUGALL) std::cout << "Written all subimages to one image" << std::endl;
		usleep(100000);
	}

	delete image_server;
	safe_close();
	scan.Stop();
	return 0;
}


