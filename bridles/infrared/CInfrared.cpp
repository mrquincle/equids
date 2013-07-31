/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CInfrared.cpp
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

#include <CInfrared.h>

#include <CMotors.h>

#include <cassert>
#include <dim1algebra.hpp>

CInfrared::CInfrared(RobotBase *robot_base, RobotBase::RobotType robot_type) {
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
}

CInfrared::~CInfrared() {
}

void CInfrared::init() {
	std::cout << "Turn off the normal LEDs for less inference" << std::endl;
	power_all(LT_NORMAL, false);

	std::cout << "Turn on the reflective IR LEDs" << std::endl;
	power_all(LT_REFLECTIVE, true);

}

//0 -- front right
//1 -- front left
//2 -- left front
//3 -- left rear
//4 -- rear left
//5 -- rear right
//6 -- right rear
//7 -- right front
int CInfrared::reflective(int i, bool offset) {
	switch (i) {
	case 0: case 1: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::FRONT));
		// pick sensor[0] or sensor[1] on this side
		return ret.sensor[1-i%2].reflective - (offset ? offset_reflective[i] : 0);
	} case 2: case 3: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::LEFT));
		return ret.sensor[1-i%2].reflective - (offset ? offset_reflective[i] : 0);
	} case 4: case 5: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::REAR));
		return ret.sensor[1-i%2].reflective - (offset ? offset_reflective[i] : 0);
	} case 6: case 7: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::RIGHT));
		return ret.sensor[1-i%2].reflective - (offset ? offset_reflective[i] : 0);
	}
	}
	assert(false);
	return -1;
}

int CInfrared::ambient(int i, bool offset) {
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

//#define LED_RED		0b00000011 /* R */
//#define LED_YELLOW	0b00001111 /* Y */
//#define LED_GREEN	0b00001100 /* G */
//#define LED_CYAN	0b00111100 /* C */
//#define LED_BLUE	0b00110000 /* B */
//#define LED_MAGENTA	0b00110011 /* M */
//#define LED_WHITE	0b00111111 /* W */
//#define LED_ORANGE	0b00000111 /* O */
//#define LED_OFF		0 /* 0 */

void CInfrared::power_all(LedType led_type, bool on) {
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
			}
		break;
	case LT_AMBIENT:
		break;
	case LT_NORMAL:
		for (int i = 0; i < 4; ++i)
			robot->SetLEDAll(i, on ? LED_RED : LED_OFF);
		break;
	}
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
void CInfrared::calibrate(bool turn_around) {
	std::cout << "Turn off the normal LEDs for calibration purposes" << std::endl;
	power_all(LT_NORMAL, false);
	power_all(LT_REFLECTIVE, true);

	if (turn_around)
		motors->setSpeeds(0, 40);

	static int32_t count = 100;
	static int32_t sleep = 75 * 1000;
	// every time step should take a while, with count=100 and sleep=20000, this takes 2 seconds
	assert((count * sleep) < 60000000); // make sure it stays under a minute
	std::cout << "We will turn for " << (double)((count * sleep) / (double)1000000) << " seconds, " <<
			"calibrating the infrared sensors" << std::endl;

	for (int t = 0; t < count; ++t) {
		for(int i=0; i < irled_count ;i++)
		{
			offset_reflective[i] += reflective(i, false);
			offset_ambient[i] += ambient(i, false);
		}
		usleep(sleep);
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

void CInfrared::get_calibration() {
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
 * The distance is calculated in the same way Wenguo does. By just averaging over a series of values and subtracting
 * the offset calculated in the calibration phase. We return the reflective LED values.
 */
int CInfrared::distance(int i) {
	hist_ambient.push(i, ambient(i));
	hist_reflective.push(i, reflective(i));
	//return hist_ambient.average(i);
	return hist_reflective.average(i);
}

/**
 * Returns a preferred angle between 0 and 360 degrees. This uses only the (smoothed average) values from the reflective
 * IR LEDs (not the ambient ones). An angle of 0 corresponds to driving straight forwards. There are 8 LEDS, so they
 * differ with 360/8=45 degrees. The LED at 0, is half that distance rotated from forwards, so at -22.5 degrees, LED 1
 * is at +22.5 degrees, LED 2 at 67.5 degrees etc.
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
void CInfrared::direction(float & angle) {
	for (int i=0; i < irled_count; i++) {
		hist_ambient.push(i, ambient(i));
		hist_reflective.push(i, reflective(i));
	}
	float beta = 360 / irled_count; // = 360/8 is 45 degrees
	float start = -beta/2.0;

	// get the averaged values from each LED
	std::vector<float> avg_values;
	avg_values.resize(irled_count, 0);
	hist_reflective.average(avg_values.begin());

	std::cout << "Avg: ";
	for (int i = 0; i < avg_values.size(); ++i) {
		std::cout << avg_values[i] << ' ';
	}
	std::cout << std::endl;

	// not every LED is as important, weight the ones in the front more than the ones at the rear
	std::vector<float> weights;
	weights.resize(irled_count, 0);
	weights[0] = 1.3;
	weights[1] = 1.3;
	weights[2] = 1.2;
	weights[3] = 1.2;
	weights[4] = 1;
	weights[5] = 1;
	weights[6] = 1.2;
	weights[7] = 1.2;

	std::transform(avg_values.begin(), avg_values.end(), weights.begin(), avg_values.begin(),
			std::multiplies<float>());

	std::cout << "Avg: ";
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

	// make 4 directions is good enough
	angle_index /= 2;

	// get from index to angle
	angle = beta * 2 * angle_index; // + start


}
