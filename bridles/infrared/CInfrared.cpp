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
	hist_reflective.set_sliding_window(10);
	hist_ambient.set_sliding_window(10);
	save_to_file = true;
}

CInfrared::~CInfrared() {
}

//0 -- front right
//1 -- front left
//2 -- left front
//3 -- left rear
//4 -- rear left
//5 -- rear right
//6 -- right rear
//7 -- right front
int CInfrared::reflective(int i) {
	switch (i) {
	case 0: case 1: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::FRONT));
		// pick sensor[0] or sensor[1] on this side
		return ret.sensor[1-i%2].reflective;
	} case 2: case 3: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::LEFT));
		return ret.sensor[1-i%2].reflective;
	} case 4: case 5: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::REAR));
		return ret.sensor[1-i%2].reflective;
	} case 6: case 7: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::RIGHT));
		return ret.sensor[1-i%2].reflective;
	}
	}
	assert(false);
	return -1;
}

int CInfrared::ambient(int i) {
	switch (i) {
	case 0: case 1: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::FRONT));
		return ret.sensor[1-i%2].ambient;
	} case 2: case 3: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::LEFT));
		return ret.sensor[1-i%2].ambient;
	} case 4: case 5: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::REAR));
		return ret.sensor[1-i%2].ambient;
	} case 6: case 7: {
		IRValues ret = robot->GetIRValues(robot->GetSide(RobotBase::RIGHT));
		return ret.sensor[1-i%2].ambient;
	}
	}
	assert(false);
	return -1;
}

/**
 * This is the same way Wenguo calibrates the IR. The only difference is that he stores them to a file, so he only needs
 * to calibrate the robot once and then he uses this configuration. Both the reflective and the ambient IR leds are
 * calibrated.
 */
void CInfrared::calibrate(bool turn_around) {
	if (turn_around)
		motors->setSpeeds(0, 30);

	static int32_t count = 100;
	static int32_t sleep = 20000;
	// every time step should take a while, with count=100 and sleep=20000, this takes 2 seconds
	assert((count * sleep) < 60000000); // make sure it stays under a minute

	for (int t = 0; t < count; ++t) {
		for(int i=0; i < irled_count ;i++)
		{
			offset_reflective[i] += reflective(i);
			offset_ambient[i] += ambient(i);
		}
		usleep(sleep);
	}

	for(int i=0;i<irled_count;i++) {
		offset_reflective[i] /= count; // take average
		offset_ambient[i] /= count; // take average
	}

	if(save_to_file) {
		for( int entity = 1; entity < optionfile.GetEntityCount(); ++entity )
		{
			const char *typestr = (char*)optionfile.GetEntityType(entity);
			if( strcmp( typestr, "Global" ) == 0 )
			{
				char default_str[64];
				for(int i=0;i<irled_count;i++)
				{
					snprintf(default_str, sizeof(default_str), "%d", offset_reflective[i]);
					optionfile.WriteTupleString(entity, "reflective_calibrated", i, default_str);
					snprintf(default_str, sizeof(default_str), "%d", offset_ambient[i]);
					optionfile.WriteTupleString(entity, "ambient_calibrated", i, default_str);
				}
			}
		}
	}

	if(type == RobotBase::KABOT)
		optionfile.Save("/flash/morph/kit_option.cfg");
	else if(type == RobotBase::ACTIVEWHEEL)
		optionfile.Save("/flash/morph/aw_option.cfg");
	else if(type == RobotBase::SCOUTBOT)
		optionfile.Save("/flash/morph/scout_option.cfg");
}

/**
 * The distance is calculated in the same way Wenguo does. By just averaging over a series of values and subtracting
 * the offset calculated in the calibration phase.
 */
int CInfrared::distance(int i) {
	//	for (int i=0; i < irled_count; i++) {
	hist_ambient.push(i, ambient(i));
	hist_reflective.push(i, reflective(i));
	//	}
	return hist_ambient.average(i);
}

/**
 * Returns a preferred angle between 0 and 360 degrees.
 */
void CInfrared::direction(float & angle) {
	for (int i=0; i < irled_count; i++) {
		hist_ambient.push(i, ambient(i));
		hist_reflective.push(i, reflective(i));
	}
	float beta = 360 / irled_count; // = 360/8 is 45 degrees
	float start = -beta/2.0;

	// get the averaged values from each LED
	std::vector<int32_t> avg_values;
	avg_values.resize(irled_count, 0);
	hist_reflective.average(avg_values.begin());

	// not every LED is as important, weight the ones in the front more than the ones at the rear
	std::vector<int32_t> weights;
	weights.resize(irled_count, 0);
	weights[0] = 3;
	weights[1] = 3;
	weights[2] = 2;
	weights[3] = 2;
	weights[4] = 1;
	weights[5] = 1;
	weights[6] = 2;
	weights[7] = 2;

	std::transform(avg_values.begin(), avg_values.end(), weights.begin(), avg_values.begin(),
			std::multiplies<int32_t>());

	// smooth the values with their neighbours
	std::vector<int32_t> values;
	values.resize(irled_count, 0);
	dobots::window_add(avg_values.begin(), avg_values.end(), values.begin(), 1);

}
