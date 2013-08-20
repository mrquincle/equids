/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CLeds.cpp
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

#include <CLeds.h>

#include <CMotors.h>

#include <cassert>
#include <dim1algebra.hpp>

CLeds::CLeds(RobotBase *robot_base, RobotBase::RobotType robot_type) {
	motors = new CMotors(robot_base, robot_type);
	type = robot_type;
	robot = robot_base;
	irled_count = 8;
	// assume bias of zero, use calibrate() if that's not okay for your robots
	offset_reflective.resize(irled_count, 0);
	offset_ambient.resize(irled_count, 0);
	hist_reflective.add(irled_count);
	hist_ambient.add(irled_count);
	window_size = 100;
	hist_reflective.set_sliding_window(window_size);
	hist_ambient.set_sliding_window(window_size);
	save_to_file = true;
//	save_to_file = false;
	if(type == RobotBase::KABOT)
		optionfile_name = "/flash/morph/kit_option.cfg";
	else if(type == RobotBase::ACTIVEWHEEL)
		optionfile_name =  "/flash/morph/aw_option.cfg";
	else if(type == RobotBase::SCOUTBOT)
		optionfile_name = "/flash/morph/scout_option.cfg";
	optionfile.Load(optionfile_name);

	// order in specific sequence to make it avoid collisions between the MSPs
	ir_query_order.resize(irled_count, 0);
	ir_query_order[0] = LL_FRONT_RIGHT;
	ir_query_order[1] = LL_REAR_LEFT;
	ir_query_order[2] = LL_LEFT_FRONT;
	ir_query_order[3] = LL_RIGHT_REAR;
	ir_query_order[4] = LL_FRONT_LEFT;
	ir_query_order[5] = LL_REAR_RIGHT;
	ir_query_order[6] = LL_RIGHT_FRONT;
	ir_query_order[7] = LL_LEFT_REAR;


}

CLeds::~CLeds() {
}

void CLeds::init() {
	std::cout << "Turn off the normal LEDs for less inference" << std::endl;
	power_all(LT_NORMAL, false);

	std::cout << "Turn on the reflective IR LEDs" << std::endl;
	power_all(LT_REFLECTIVE, true);

	int board_count = 4;
	board_running.resize(board_count, 0);
	for (int i = 0; i < board_count; i++) {
		board_running[i] = robot->IsBoardRunning(i);
	}
}

//0 -- front right
//1 -- front left
//2 -- left front
//3 -- left rear
//4 -- rear left
//5 -- rear right
//6 -- right rear
//7 -- right front
int CLeds::reflective(int i, bool offset) {
	int value = 0;
	int side;
	switch (i) {
	case 0: case 1: {
		side = robot->GetSide(RobotBase::FRONT);
		break;
	} case 2: case 3: {
		side = robot->GetSide(RobotBase::LEFT);
		break;
	} case 4: case 5: {
		side = robot->GetSide(RobotBase::REAR);
		break;
	} case 6: case 7: {
		side = robot->GetSide(RobotBase::RIGHT);
		break;
	}
	}

	if (board_running[side]) {
		IRValues ret = robot->GetIRValues(side);
		value = ret.sensor[1-i%2].reflective - (offset ? offset_reflective[i] : 0);
	}
//	std::cout << "Reflective value [" << i << "]: " << value << std::endl;
	return value;
}

int CLeds::ambient(int i, bool offset) {
	return 0;
	switch (i) {
	case 0: case 1: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::FRONT));
		return ret.sensor[1-i%2].ambient - (offset ? offset_ambient[i] : 0);
	} case 2: case 3: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::LEFT));
		return ret.sensor[1-i%2].ambient - (offset ? offset_ambient[i] : 0);
	} case 4: case 5: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::REAR));
		return ret.sensor[1-i%2].ambient - (offset ? offset_ambient[i] : 0);
	} case 6: case 7: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::RIGHT));
		return ret.sensor[1-i%2].ambient - (offset ? offset_ambient[i] : 0);
	}
	}
	assert(false);
	return -1;
}

// from irobot/InterfaceTypes.h

//#define IRLED0	0x1
//#define IRLED1	0x2
//#define IRLED2	0x4
//#define IRPULSE0	0x1
//#define IRPULSE1	0x2
//#define IRPULSE2	0x4
//#define IRPULSE3	0x8
//#define IRPULSE4	0x10
//#define IRPULSE5	0x20

void CLeds::power_all(LedType led_type, bool on) {
	switch (led_type) {
	case LT_REFLECTIVE:
		if (on)
			for (int i = 0; i < 4; ++i) {
				// the IR mode is set to off, but the LEDs will be turned on in a different manner, with SetIRPulse
				robot->SetIRMode(i, IRLEDOFF);
				// set all the LEDs to pulsing
			    robot->SetIRPulse(i, IRPULSE0|IRPULSE1|IRPULSE2|IRPULSE3|IRPULSE4|IRPULSE5);
			    //  enable receiving LED (?)
		        robot->SetIRRX(i, true);

		        // calibrate IR
		        robot->CalibrateIR(i);
		        usleep(100000);
			}
		break;
	case LT_AMBIENT:
		std::cerr << "Are these not on by default?" << std::endl;
		break;
	case LT_NORMAL:
		for (int i = 0; i < 4; ++i)
			robot->SetLEDAll(i, on ? LED_RED : LED_OFF);
		break;
	}
}

void CLeds::color(LedColor color) {
	uint8_t c;
	switch (color) {
	case LC_RED:          c = 0b00000011; break;
	case LC_YELLOW:       c = 0b00001111; break;
	case LC_GREEN:        c = 0b00001100; break;
	case LC_CYAN:         c = 0b00111100; break;
	case LC_BLUE:         c = 0b00110000; break;
	case LC_MAGENTA:      c = 0b00110011; break;
	case LC_WHITE:        c = 0b00111111; break;
	case LC_ORANGE:       c = 0b00000111; break;
	case LC_OFF: case LC_ENUM_SIZE: default:
		c = 0b00000000; break;
	}
	for (int i = 0; i < 4; ++i)
		robot->SetLEDAll(i, c);
}

// in FSM state WAITING
//switch on proximity ir leds and ir pulsing
//for(uint8_t i=0; i< NUM_DOCKS; i++)
//    SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);

/**
 * This is the same way Wenguo calibrates the IR. The only difference is that he stores them to a file, so he only needs
 * to calibrate the robot once and then he uses this configuration. Both the reflective and the ambient IR leds are
 * calibrated.
 */
void CLeds::calibrate(bool turn_around) {
	std::cout << "Turn off the normal LEDs for calibration purposes" << std::endl;
	power_all(LT_NORMAL, false);
	power_all(LT_REFLECTIVE, true);

	if (turn_around)
		motors->setSpeeds(0, 40);

	static int32_t count = 100;
	static int32_t between_sensors_sleep = 10 * 1000; // 0.01 sec * 2 = 0.02 sec * 8 = +/- 0.1 sec
	static int32_t between_rounds_sleep = 1000;
	static int32_t sleep = 2*between_sensors_sleep * irled_count + between_rounds_sleep;
	// every time step should take a while, with count=100 and sleep=20000, this takes 2 seconds
	std::cout << "We will turn for " << (double)((count * sleep) / (double)1000000) << " seconds, " <<
			"calibrating the infrared sensors" << std::endl;
	assert((count * sleep) < 60000000); // make sure it stays under a minute

	for (int t = 0; t < count; ++t) {
		for(int i=0; i < irled_count ;i++)
		{
			offset_reflective[i] += reflective(i, false);
			usleep(between_sensors_sleep);
			offset_ambient[i] += ambient(i, false);
			usleep(between_sensors_sleep);
		}
		usleep(between_rounds_sleep);
	}

	for(int i=0;i<irled_count;i++) {
		offset_reflective[i] /= count; // take average
		offset_ambient[i] /= count; // take average
	}

	// by default save to file
	if(save_to_file) {
		int entity = optionfile.LookupEntity("Global");
		if (entity < 0) {
			std::cerr << "The class \"worldfile\" is too basic to add entities or properties. " <<
					"You have to do that yourself manually in " << optionfile_name << std::endl;
		} else {
			char default_str[64];
			for(int i=0;i<irled_count;i++) {
				snprintf(default_str, sizeof(default_str), "%d", offset_reflective[i]);
				optionfile.WriteTupleString(entity, "reflective_calibrated", i, default_str);
				snprintf(default_str, sizeof(default_str), "%d", offset_ambient[i]);
				optionfile.WriteTupleString(entity, "ambient_calibrated", i, default_str);
			}
			optionfile.Save(optionfile_name);
		}
	}

	power_all(LT_REFLECTIVE, false);
	std::cout << "Turn the normal LEDs back on" << std::endl;
	power_all(LT_NORMAL);
}

void CLeds::get_calibration() {
	optionfile.Load(optionfile_name);

	int entity = optionfile.LookupEntity("Global");
	if (entity < 0) {
		std::cerr << "The option file does not have an entity \"Global\"" << std::endl;
		return;
	}

	std::string property_name;
	property_name = "reflective_calibrated";
	if( CProperty* prop = optionfile.GetProperty( entity, property_name.c_str() ) ) {
		for(int i=0;i<irled_count;i++) {
			offset_reflective[i] = atoi(optionfile.GetPropertyValue(prop, i));
		}
	} else {
		std::cerr << "No property called " << property_name << std::endl;
	}
	property_name = "ambient_calibrated";
	if( CProperty* prop = optionfile.GetProperty( entity, property_name.c_str() ) ) {
		for(int i=0;i<irled_count;i++) {
			offset_ambient[i] = atoi(optionfile.GetPropertyValue(prop, i));
		}
	} else {
		std::cerr << "No property called " << property_name << std::endl;
	}

	std::cout << "Calibrated values for ambient sensors:";
	for(int i=0;i<irled_count;i++) {
		std::cout << offset_ambient[i] << ' ';
	}
	std::cout << std::endl;
}

/**
 * Returns in 2000 milliseconds times the number of IR leds, so 0.016 seconds.
 */
void CLeds::update() {
	for (int i=0; i < irled_count; i++) {
		int j = ir_query_order[i];
//		hist_ambient.push(j, ambient(j));
//		usleep(20000);
		hist_reflective.push(j, reflective(j));
		usleep(1000);
	}
	// verbose
//	std::cout << "Avg[0]: " << hist_reflective.average(0) << std::endl;
//	std::cout << "Avg[1]: " << hist_reflective.average(1) << std::endl;
}

void CLeds::update(int i) {
	hist_ambient.push(i, ambient(i));
	usleep(1000);
	hist_reflective.push(i, reflective(i));
	usleep(1000);
}

/**
 * The distance is calculated in the same way Wenguo does. By just averaging over a series of values and subtracting
 * the offset calculated in the calibration phase. We return the reflective LED values.
 */
int CLeds::distance(int i) {
	update(i);
	//return hist_ambient.average(i);
	return hist_reflective.average(i);
}

/**
 * Returns a sign for the speed and returns the radius so that the robot drives away from closest point.
 * @param sign_speed         given(!) a speed, this function adds a sign
 * @param radius             radius will be returned
 *
 * Internal to this function this uses the IR LEDs (not the ambient ones). An angle of 0 corresponds to driving
 * straightforward. There are 8 LEDS, so they differ with 360/8=45 degrees. The LED at 0, is half that distance rotated
 * from forwards, so at -22.5 degrees, LED 1 is at +22.5 degrees, LED 2 at 67.5 degrees etc.
 *
 *       forward
 *     __1_____0__
 *    |           |
 *   2|           |7
 *    |           |
 *   3|           |6
 *    |___________|
 *       4     5
 *
 * The LEDs in the forward direction have larger weights, so the robot is tempted to go forwards. The final angle that
 * rolls out corresponds with the angle with the largest "vote". The robot will always go in a direction the opposite
 * of the highest value (which is assumed to be the most spacious direction).
 */
void CLeds::direction(int & sign_speed, int & radius) {
	update();

	float beta = 360 / irled_count; // = 360/8 is 45 degrees
	float start = -beta/2.0;

	// get the averaged values from each LED
	std::vector<float> avg_values;
	avg_values.resize(irled_count, 0);
	hist_reflective.average(avg_values.begin());

	std::cout << "Averages: ";
	for (int i = 0; i < avg_values.size(); ++i) {
		std::cout << avg_values[i] << ' ';
	}
	std::cout << std::endl;

	// not every LED is as important, weight the ones in the front more than the ones at the rear
	std::vector<float> weights;
	weights.resize(irled_count, 0);
	weights[0] = 1.7;
	weights[1] = 1.7;
	weights[2] = 1.4;
	weights[3] = 1.2;
	weights[4] = 1;
	weights[5] = 1;
	weights[6] = 1.4;
	weights[7] = 1.2;

	// multiply values with above weights
	std::transform(avg_values.begin(), avg_values.end(), weights.begin(), avg_values.begin(), std::multiplies<float>());

	std::cout << "Weighted averages: ";
	for (int i = 0; i < avg_values.size(); ++i) {
		std::cout << avg_values[i] << ' ';
	}
	std::cout << std::endl;

	// smooth the values with their neighbours
	std::vector<int32_t> values;
	values.resize(irled_count, 0);
	dobots::window_add(avg_values.begin(), avg_values.end(), values.begin(), 1);

	// get the "winning" LED
	std::vector<int32_t>::iterator iter;
	iter = std::max_element(values.begin(), values.end());
	int angle_index = std::distance(values.begin(), iter);

	// highest value is most dangerous, so pick the opposite side
	angle_index = (angle_index + 4) % 8;

	sign_speed = abs(sign_speed);

	// To turn quick you need a small radius
	int turn_quick = 20;

	// To turn slow (or go straight) you need a large radius
	int turn_slow = 1000;

	switch (angle_index) {
	case 0: {
		sign_speed = -sign_speed;
		radius = turn_slow;
		break;
	}
	case 1: {
		sign_speed = -sign_speed;
		radius = -turn_slow;
		break;
	}
	case 2: {
		sign_speed = -sign_speed;
		radius = -turn_quick;
		break;
	}
	case 7: {
		sign_speed = -sign_speed;
		radius = turn_quick;
		break;
	}
	case 3: {
		//sign_speed = +sign_speed;
		radius = -turn_quick;
		break;
	}
	case 6: {
		radius = turn_quick;
		break;
	}
	case 4: {
		radius = -turn_slow;
		break;
	}
	case 5: {
		radius = turn_slow;
		break;
	}

	}
	// make 4 directions is good enough
//	angle_index /= 2;

	// get from index to angle
//	angle = beta * 2 * angle_index; // + start

}

bool CLeds::collision() {
	update(LL_FRONT_LEFT);
	update(LL_FRONT_RIGHT);

	int threshold = 50;

	//return hist_ambient.average(i);
	int collision = 0;
	int avg_fl = hist_reflective.average((int)LL_FRONT_LEFT) ;
	int avg_fr = hist_reflective.average((int)LL_FRONT_RIGHT) ;
	std::cout << avg_fl << " and " << avg_fr << std::endl;
	// values go up if there is something close
	if (avg_fr > threshold) {
			return true;
		}
//	if (avg_fl + avg_fr > threshold) {
//		return true;
//	}
	return false;
}
