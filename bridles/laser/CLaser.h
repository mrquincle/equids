/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Control the laser (which is an actuator, not a sensor)
 * @file CLaser.h
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

#ifndef CLASER_H_
#define CLASER_H_

#include <IRobot.h>

/* **************************************************************************************
 * Interface of CLaser
 * **************************************************************************************/

/**
 * CLaser class uses the "irobot" library to send commands to the laser.
 */
class CLaser {
public:
	//! Constructor CLaser
	CLaser(RobotBase *robot_base, RobotBase::RobotType robot_type);

	//! Destructor ~CLaser
	virtual ~CLaser();

	//! Is the laser on (returns true) or off (false)
	bool IsOn();

	//! Turn laser on
	void On();

	//! Turn laser off
	void Off();

	//! If it is off, turn it on, and the other way around
	void Toggle();

	//! Set prefix for log messages
	inline void setLogPrefix(std::string log_prefix) {
		this->log_prefix = log_prefix + "CCamera: ";
	}
protected:

private:
	//! Status is either on or off
	bool status_on;

	//! Reference to the robot class and type in the "irobot" library
	RobotBase *robot;
	RobotBase::RobotType robot_type;

	std::string log_prefix;
};

#endif /* CLASER_H_ */
