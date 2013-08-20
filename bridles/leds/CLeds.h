/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief General file to control and read from infrared and normal LEDs
 * @file CLeds.h
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

#ifndef CLEDS_H_
#define CLEDS_H_

#include <IRobot.h>
#include <CMotors.h>

#include <vector>

#include <worldfile.h>

#include <CMultiHistogram.h>

enum LedType { LT_REFLECTIVE, LT_AMBIENT, LT_NORMAL, LT_ENUM_SIZE };

enum LedColor { LC_RED, LC_YELLOW, LC_GREEN, LC_CYAN, LC_BLUE, LC_MAGENTA, LC_WHITE, LC_ORANGE, LC_OFF, LC_ENUM_SIZE };

/**
 * Scout:
 *
 *       forward
 *     __1_____0__
 *    |           |
 *   2|           |7
 *    |           |
 *   3|           |6
 *    |___________|
 *       4     5
 */
enum LedLocation {
	LL_FRONT_RIGHT = 0,
	LL_FRONT_LEFT = 1,
	LL_LEFT_FRONT = 2,
	LL_LEFT_REAR = 3,
	LL_REAR_LEFT = 4,
	LL_REAR_RIGHT = 5,
	LL_RIGHT_REAR = 6,
	LL_RIGHT_FRONT = 7
};

class CLeds {
public:
	CLeds(RobotBase *robot_base, RobotBase::RobotType robot_type);

	~CLeds();

	void init();

	void calibrate(bool turn_around = true);

	void get_calibration();

	int reflective(int i, bool offset=true);

	int ambient(int i, bool offset=true);

	//! Update all sensors (includes waiting time)
	void update();

	//! Update sensor values of sensor with index i
	void update(int i);

	//! Get distance measurement
	int distance(int i);

	//! Get control to drive in direction of no collisions
	void direction(int & sign_speed, int & radius);

	//! Just a collision signal for the front leds
	bool collision();

	//! Power on the given LED type
	void power_all(LedType led_type, bool on=true);

	//! Set to specific color
	void color(LedColor color);

	//! Useful to get distances from the infrared LEDs
	inline int get_window_size() { return window_size; }
private:

	CMotors *motors;

	RobotBase::RobotType type;

	RobotBase *robot;

	int irled_count;

	std::vector<int> ir_query_order;

	std::vector<int> board_running;

	bool save_to_file;

	std::vector<int32_t> offset_reflective;

	std::vector<int32_t> offset_ambient;

	CMultiHistogram<int32_t, int32_t> hist_reflective;

	CMultiHistogram<int32_t, int32_t> hist_ambient;

	int window_size;

	Worldfile optionfile;

	std::string optionfile_name;
};

#endif /* CLEDS_H_ */

