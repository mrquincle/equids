/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file AvoidIRController.h
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
 * @date      Aug 16, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef AVOIDIRCONTROLLER_H_
#define AVOIDIRCONTROLLER_H_

#include <string>
#include <CMessageServer.h>
#include <IRobot.h>

#include <CMotors.h>
#include <CLeds.h>

#include <cassert>

//! The name of the controller can be used for controller selection
static const std::string NAME = "AvoidInfrared";

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

class AvoidIRController {
public:
	AvoidIRController();

	virtual ~AvoidIRController();

	void parsePort(int argc, char **argv);

	void initServer();

	void initRobot();

	void initRobotPeriphery();

	void pause();

	void start();

	void acknowledge();

	void calibrate();

	void get_calibration();

	void tick();

	void signal_end();

	void graceful_end();

	inline std::string getPort() { return port; }

	inline CMessageServer *getServer() { return server; }

	//! Actually ugly that message is copied here, should have been by const and by reference
	inline const CMessage & getMessage() { assert (server != NULL); return server->getMessage(); }
private:

	std::string port;

	CMessageServer *server;

	RobotBase *robot;

	RobotBase::RobotType robot_type;

	CMotors *motors;

	CLeds *leds;
};


#endif /* AVOIDIRCONTROLLER_H_ */
