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

#include <syslog.h>
#include <vector>

#include <worldfile.h>
#include <pthread.h>

#include <CMultiHistogram.h>

enum LedType { LT_REFLECTIVE, LT_AMBIENT, LT_PROXIMITY, LT_NORMAL, LT_ENUM_SIZE };

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

/**
 * Everything around the infrared and other types of leds on the robot.
 */
class CLeds {
public:
	//! Construct everything
	CLeds(RobotBase *robot_base, RobotBase::RobotType robot_type);

	//! Deallocate everything
	~CLeds();

	//! Initialize the leds, calibrate the MSP stuff
	bool init();

	//! Do the calibration now, if turn_around use the wheels to turn around
	void calibrate(bool turn_around = true);

	//! Get previously calibrated values
	void get_calibration();

	//! Map from led index to side index
	int led_index_to_side(int i);

	//! Get actual sensor values
	void sample();

	//! Get the reflective led with index i
	int reflective(int i, bool offset=true);

	//! Get the ambient led with index i
	int ambient(int i, bool offset=true);

	//! Get the proximity led with index i
	int proximity(int i, bool offset=true);

	//! Clear all histogram info
	void reset();

	//! Update all sensors (includes waiting time)
	void update();

	//! Update sensor values of sensor with index i
	void update(int i);

	//! Get distance measurement
	int distance(int i);

	//! Get control to drive in direction of no collisions
	void direction(int & sign_speed, int & radius);

	//! Just a collision signal for the front leds
	bool collision(LedLocation loc=LL_FRONT_LEFT, int adjust_treshold=0);

	//! Power on the given LED type
	void power_all(LedType led_type, bool on=true);

	//! Set to specific color
	void color(LedColor color);

	//! Toggle to specific color
	void colorToggle(LedColor color);

	//! Useful to get distances from the infrared LEDs
	inline int get_window_size() { return window_size; }

	//! Set verbosity
	inline void setVerbosity(char verbosity) { log_level = verbosity; }

	//! Thread that continuously checks if there has been a message received over infrared
    static void *IRCommTxThread(void* leds);

    //! Only get indication that "a" message is received, not which message
	bool message_received();

	//! Send this message to a random side of the robot
	void send_message(const std::string & msg);

	//! Encounter of another robot
	bool encounter();

	//! Set prefix for log messages
	inline void setLogPrefix(std::string log_prefix) {
		this->log_prefix = log_prefix + "CLeds: ";
		std::string motor_log_prefix = log_prefix + "From CLeds, ";
		if (motors) motors->setLogPrefix(motor_log_prefix);
	}

private:

	CMotors *motors;

	RobotBase::RobotType type;

	RobotBase *robot;

	int irled_count;

	bool colored;


	std::vector<int> board_running;

	bool save_to_file;

	int board_count;

	std::vector<int32_t> offset_reflective;

	std::vector<int32_t> offset_ambient;

	std::vector<int32_t> offset_proximity;

	std::vector<int32_t> variance_reflective;

	std::vector<int32_t> variance_ambient;

	std::vector<int32_t> variance_proximity;

	//! Store the ir-values
	std::vector<IRValues> ir_values;

	CMultiHistogram<int32_t, int32_t> hist_reflective;

	CMultiHistogram<int32_t, int32_t> hist_ambient;

	CMultiHistogram<int32_t, int32_t> hist_proximity;

	int window_size;

	Worldfile optionfile;

	std::string optionfile_name;

	char log_level;

    pthread_t ircomm_rx_thread;

    bool enable_reflective;

    bool enable_ambient;

    bool enable_proximity;

public:
	pthread_mutex_t ir_rx_mutex;

	bool receiving_messages;

	bool message_recv;

	long int messages_sent;

	std::string log_prefix;
};

#endif /* CLEDS_H_ */

