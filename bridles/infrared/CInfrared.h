/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CInfrared.h
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
 * @date      Jul 4, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CINFRARED_H_
#define CINFRARED_H_

#include <IRobot.h>
#include <CMotors.h>

#include <vector>

#include <worldfile.h>

#include <CMultiHistogram.h>

enum LedType { LT_REFLECTIVE, LT_AMBIENT, LT_NORMAL, LT_ENUM_SIZE };

class CInfrared {
public:
	CInfrared(RobotBase *robot_base, RobotBase::RobotType robot_type);

	~CInfrared();

	void init();

	void calibrate(bool turn_around = true);

	void get_calibration();

	int reflective(int i, bool offset=true);

	int ambient(int i, bool offset=true);

	//! Get distance measurement
	int distance(int i);

	//! Get control to drive in direction of no collisions
	void direction(int & sign_speed, int & radius);

	void power_all(LedType led_type, bool on=true);

	inline int get_window_size() { return window_size; }
private:
	CMotors *motors;

	RobotBase::RobotType type;

	RobotBase *robot;

	int irled_count;

	bool save_to_file;

	std::vector<int32_t> offset_reflective;

	std::vector<int32_t> offset_ambient;

	CMultiHistogram<int32_t, int32_t> hist_reflective;

	CMultiHistogram<int32_t, int32_t> hist_ambient;

	int window_size;

	Worldfile optionfile;

	std::string optionfile_name;
};


#endif /* CINFRARED_H_ */

