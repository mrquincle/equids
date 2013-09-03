/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Infrared avoidance controller
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

#include <CMotors.h>
#include <CLeds.h>

#include <CController.h>

//! The name of the controller can be used for controller selection
static const std::string NAME = "AvoidInfrared";

//! Convenience function for printing to standard out
#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

/**
 * Use this class as a template class for other controllers if you want to quickly prototype your own controller. The
 * basic functionality is probably the same, except for the tick() function where you can use your own stuff.
 */
class AvoidIRController: public CController {
public:
	//! Constructor
	AvoidIRController();

	//! Destructor
	virtual ~AvoidIRController();

	//! Initialize the periphery, probably best be done when you got the control from the jockey framework
	void initRobotPeriphery();

	//! Controller specific function for calibration procedure
	void calibrate();

	//! Controller specific function for getting calibration values
	void get_calibration();

	//! Just print sensor values, do nothing
	void print();

	//! Controller specific code
	void tick();

	//! Signal the end of the controller by using LEDs
	void signal_end();

	//! Stop the controller by proper deallocation
	void graceful_end();

private:
	//! Specific bridles to be used
	CMotors *motors;
	CLeds *leds;
};


#endif /* AVOIDIRCONTROLLER_H_ */
