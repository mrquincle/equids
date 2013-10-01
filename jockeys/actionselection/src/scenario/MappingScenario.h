/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Scenario for exploration after the map has been build up. It uses laser to label objects in the environment.
 * @file LaserExplorationScenario.h
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * Copyright (c) 2013 Anne C. van Rossum <anne@almende.org>
 *
 * @author    Robert Pěnička
 * @date      Aug 18, 2013
 * @project   Replicator 
 * @company   CTU in Prague
 * @case      Sensor fusion
 */

#ifndef MAPPINGSCENARIO_H_
#define MAPPINGSCENARIO_H_

#ifndef ENABLE_MAPPING_SCENARIO
#define ENABLE_MAPPING_SCENARIO
#endif

#ifdef ENABLE_MAPPING_SCENARIO

#include <CScenario.h>

/**
 * After starting we perform random exploration. Later on we will need to replace this by going to position indicated
 * as interesting by the mapping jockey.
 */

class MappingScenario: public CScenario {
public:
#ifndef __TState__
#define __TState__
typedef enum {
	S_START = 0,
	S_CALIBRATE_ODOMETRY,
	S_BUILD_MAP,
	S_WAIT_FOR_MAP_FROM_OTHERS,
	S_DOCK_SOCKET,
	S_DOCKED,
	S_STREAM_VIDEO,
	S_MAPPING_DETECTION,
	S_UBIPOS,
	S_REMOTE_CONTROL,
	S_DOCK_NOW,
	S_ORGANISM_REMOTECONTROL,
	S_ORGANISM_DOCKING,
	S_REMOTE_CONTROLLED_BY_LEADER,
	S_LEADER_OF_ORGANISM_DOCKING,
	S_LEADER_OF_ORGANISM_REMOTECONTROL,
	S_QUIT
} TState;
#endif
	//! Construct and get the CEquids object.
	MappingScenario(CEquids * equids);

	//! Deallocate (nothing)
	~MappingScenario();

	//! Initializing all the jockeys and starting the first one
	bool Init();

	//! Run indefinitely
	void Run();

	// Define the jockeys for this scenario
	jockey_id J_MAPPING;
	jockey_id J_CAMERADETECTION;
	jockey_id J_POSITION;
	jockey_id J_MOTORCALIBRATION;
	jockey_id J_ZBMESSENGER;
	jockey_id J_DRIVE_TO_POSITION;
	jockey_id J_DOCK_SOCKET;
	jockey_id J_REMOTE_CONTROL;
	jockey_id J_ORGANISM_CONTROL;

private:
	//! State for the state machine in Run()
	TState state;
	jockey_id lastActiveJockey;
	int sleepTime;
	//! A raised quit flag will drop out of the while loop in Run()
	bool quit;
};

#endif // ENABLE_MAPPING_SCENARIO


#endif /* MAPPINGSCENARIO_H_ */
