/**
 * @brief Control the laser (actually an actuator, not a sensor)
 * @file CLaser.h
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Jul 30, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
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
protected:

private:
	//! Status is either on or off
	bool status_on;

	//! Reference to the robot class and type in the "irobot" library
	RobotBase *robot_base;
	RobotBase::RobotType robot_type;
};

#endif /* CLASER_H_ */
