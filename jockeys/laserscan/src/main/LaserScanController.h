/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file LaserScanController.h
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

#ifndef LASERSCANCONTROLLER_H_
#define LASERSCANCONTROLLER_H_

#include <CController.h>

#include <CLaserScan.h>
#include <CImageServer.h>
#include <CRawImage.h>
#include <CMotors.h>

#include <semaphore.h>
#include <vector>

#include <messageDataType.h>

/**
 * Controller for the laser scan.
 */
class LaserScanController: public CController {
public:
	LaserScanController();

	virtual ~LaserScanController();

	void initRobotPeriphery();

	void tick();

	//! Overload pause to also stop the motors
	void pause();

	void startVideoStream(std::string port);

	void stopVideoStream();

	void testCamera();

	void sendDetectedObject(MappedObjectPosition &position);

	void motorCommand(MotorCommand &motorCommand);
private:
	CLaserScan *scan;

	sem_t *imageSem;

	CImageServer* image_server;

	std::vector<CRawImage*> images;

	Patch patch[4];

	CRawImage *mosaic_image;

	bool streaming;

	CMotors *motors;

	bool create_mosaic;

	bool initialized_periphery;
};


#endif /* LASERSCANCONTROLLER_H_ */
