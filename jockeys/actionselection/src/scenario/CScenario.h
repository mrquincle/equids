/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CScenario.h
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

#ifndef CSCENARIO_H_
#define CSCENARIO_H_

#include <CEquids.h>

typedef int jockey_id;

/**
 * Subclass this scenario class to build your own.
 */
class CScenario {
public:
	CScenario(CEquids * equids);

	virtual ~CScenario();

	/**
	 * Overload this function to initialize your jockeys, return false if you want to quit program because the
	 * initialization was not successful.
	 */
	virtual bool Init() = 0;

	/**
	 * This is the main function for your program. Copy your original main to the content of this function in a
	 * separate YourScenario.cpp file. You have to quit this function yourself, after which your controller ends.
	 */
	virtual void Run() = 0;

protected:

	CEquids * equids;
};


#endif /* CSCENARIO_H_ */
