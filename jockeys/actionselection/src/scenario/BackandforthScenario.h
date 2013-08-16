/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file BackandforthScenario.h
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

#ifndef BACKANDFORTHSCENARIO_H_
#define BACKANDFORTHSCENARIO_H_

#include <CScenario.h>

typedef enum
{
	S_FORTH = 0,
	S_BACK,
	S_RECRUITING,
	S_QUIT
} TState;

/**
 * A simple scenario to show the switching between jockeys
 */
class BackandforthScenario: public CScenario {
public:
	BackandforthScenario(CEquids * equids);

	~BackandforthScenario();

	bool Init();

	void Run();

	// define the jockeys for this scenario
	jockey_id J_BACK;

	jockey_id J_FORTH;

	jockey_id J_WENGUO;
private:

	bool quit;

	TState state;

	int num;

	int cnt;
};


#endif /* BACKANDFORTHSCENARIO_H_ */
