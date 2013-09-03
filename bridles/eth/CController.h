/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Basic controller (or jockey in jockey framework)
 * @file CController.h
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

#ifndef CCONTROLLER_H_
#define CCONTROLLER_H_

#include <string>
#include <CMessageServer.h>
#include <IRobot.h>
#include <cassert>

/**
 * Controller base class to be used in jockey framework. Functions that require implementation are initRobotPeriphery()
 * with respect to for example initialization of leds, lasers, motors, and alike, and tick() which does the actual hard
 * work. Of course subclasses can have other specialized functions.
 */
class CController {
public:
	CController();

	virtual ~CController();

	//! Parse default commands from command line, first should be jockey framework port
	void parsePort(int argc, char **argv);

	//! Initialize connection to action selection module in jockey framework
	void initServer();

	//! Initialize the robot class itself, should not reset MSPs and that kind of stuff
	void initRobot();

	//! Initialize the periphery, probably best be done when you got the control from the jockey framework
	virtual void initRobotPeriphery() = 0;

	//! Pause in jockey framework
	void pause();

	//! Start in jockey framework
	void start();

	//! Ack in jockey framework
	void acknowledge();

	//! Controller specific code
	virtual void tick() = 0;

	//! Port used in jockey framework
	inline std::string getPort() { return port; }

	//! Server of jockey framework
	inline CMessageServer *getServer() { return server; }

	//! Actually ugly that message is copied here, should have been by const and by reference
	inline const CMessage & getMessage() { assert (server != NULL); return server->getMessage(); }

	//! Set verbosity
	inline void setVerbosity(char verbosity) {
		log_level = verbosity;
	}

protected:
	//! Port of jockey framework
	std::string port;

	//! To send messages in jockey framework
	CMessageServer *server;

	//! Reference to irobot
	RobotBase *robot;

	//! Type of robot (Scout, ActiveWheel, Backbone)
	RobotBase::RobotType robot_type;

	//! Robot id as set in sr_id envionmental variable
	int robot_id;

	//! Debug state
	char log_level;

};


#endif /* CCONTROLLER_H_ */
