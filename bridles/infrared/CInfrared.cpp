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

CInfrared::CInfrared(RobotBase *robot_base, RobotBase::RobotType robot_type) {
	motors = new CMotors(robot_base, robot_type);
	type = robot_type;
	robot = robot_base;
	irled_count = 8;
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

void CInfrared::calibrate() {
	motors->setSpeeds(0, 30);

	static int32_t temp1[8]={0,0,0,0,0,0,0,0};
	static int32_t temp2[8]={0,0,0,0,0,0,0,0};
	assert (irled_count <= 8);
	static int32_t count = 100;

	for (int t = 0; t < count; ++t) {
		for(int i=0; i< irled_count ;i++)
		{
			temp1[i] += reflective(i);
//			temp2[i] += ambient[i];
		}
	}

	for(int i=0;i<irled_count;i++) {
		temp1[i] /= count; // take average
	}

//	if(save_to_file) {
//		for( int entity = 1; entity < optionfile->GetEntityCount(); ++entity )
//		{
//			const char *typestr = (char*)optionfile->GetEntityType(entity);
//			if( strcmp( typestr, "Global" ) == 0 )
//			{
//				char default_str[64];
//				for(int i=0;i<NUM_IRS;i++)
//				{
//					snprintf(default_str, sizeof(default_str), "%d", para.reflective_calibrated[i]);
//					optionfile->WriteTupleString(entity, "reflective_calibrated", i, default_str);
//					snprintf(default_str, sizeof(default_str), "%d", para.ambient_calibrated[i]);
//					optionfile->WriteTupleString(entity, "ambient_calibrated", i, default_str);
//				}
//			}
//		}
//	}
//
//	if(type == RobotBase::KABOT)
//		optionfile->Save("/flash/morph/kit_option.cfg");
//	else if(type == RobotBase::ACTIVEWHEEL)
//		optionfile->Save("/flash/morph/aw_option.cfg");
//	else if(type == RobotBase::SCOUTBOT)
//		optionfile->Save("/flash/morph/scout_option.cfg");

}

