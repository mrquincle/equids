/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Drive the robot's motors
 * @file CMotors.h
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
 * @date      Jun 6, 2013
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CMOTORS_H_
#define CMOTORS_H_

#include <IRobot.h>

/* *********************************************************************************************************************
 * Interface of CMotors
 * ********************************************************************************************************************/

/**
 * The number and location of the motors is different for each robot. However, our controllers should not be bothered
 * by that. Hence, this is a little wrapper that converts commands that set the speeds into specific commands to the
 * wheels.
 */
class CMotors {
public:
  CMotors(RobotBase *robot_base, RobotBase::RobotType robot_type);

  ~CMotors();

  void init();

  void setSpeeds(int forward, int turn);

  void halt();

  void go();

private:
	//! Reference to the robot class and type
	RobotBase *robot_base;
	RobotBase::RobotType robot_type;
};

#endif /* CMOTOR_H_ */
