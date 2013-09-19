/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Scenario for exploration after the map has been build up. It uses laser to label objects in the environment.
 * @file LaserExplorationScenario.h
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

#ifndef LASEREXPLORATIONSCENARIO_H_
#define LASEREXPLORATIONSCENARIO_H_

#include <CScenario.h>

/**
 * The scenario implements the overall Grand Challenge 1. It contains mapping, exploration (which puts on the map where
 * there are small steps versus walls), and eventually also where the specially colored power outlets are. Then there
 * is a phase of recruiting from an organism that subsequently assembles to a form with 5 robots. Subsequently there is
 * a macro-locomotion phase where this robot organism moves around towards a small step. At this step climbing is
 * initiated. When the step is climbed the robot organism is disassembled from 5 robots to 3 robots. Now, this robot is
 * able to recognize the socket and dock to it by using vision. Then the scenario ends.
 */
class LaserExplorationScenario: public CScenario {
public:
	/**
	 * After starting we perform random exploration. Later on we will need to replace this by going to position indicated
	 * as interesting by the mapping jockey.
	 */
	typedef enum
	{
		S_START,
		S_RANDOM_EXPLORATION,              // randomly explore, using infrared for collision avoidance
		S_RECOGNITION,                     // recognize (height of) an object, small obstacle like a step, or a wall
		S_QUIT
	} TState;

	//! Construct and get the CEquids object.
	LaserExplorationScenario(CEquids * equids);

	//! Deallocate (nothing)
	~LaserExplorationScenario();

	//! Initializing all the jockeys and starting the first one
	bool Init();

	//! Run indefinitely
	void Run();

	// Define the jockeys for this scenario
	jockey_id J_RANDOM_EXPLORATION;
	jockey_id J_LASER_RECOGNITION;
	jockey_id J_POSITION;

private:
	//! State for the state machine in Run()
	TState state;

	//! A raised quit flag will drop out of the while loop in Run()
	bool quit;
};


#endif /* LASEREXPLORATIONSCENARIO_H_ */
