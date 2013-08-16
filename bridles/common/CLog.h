/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CLog.h
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
 * @date      Aug 12, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CLOG_H_
#define CLOG_H_

#define ASSERT(condition) { \
	if(!(condition)){ \
		std::cerr << "ASSERT FAILED: " << #condition << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; \
		assert(condition); \
	} \
	}

#define ASSERT_EQUAL(x,y) \
	if (x != y) { \
		std::cerr << "ASSERT FAILED: " << #x << " != " << #y ", specifically: " << x << " != " << y << " @ "  \
		<< __FILE__ << " (" << __LINE__ << ")" << std::endl; \
		assert(x == y); \
	}


#endif /* CLOG_H_ */
