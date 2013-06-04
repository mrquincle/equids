/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Fusion (used to fuse camera and laser data)
 * @file CFusion.h
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
 * @author    Ted P. Schmidt
 * @author    Anne C. van Rossum
 * @date      Dec 14, 2010
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Testing
 */

#ifndef CFUSION_H_
#define CFUSION_H_

#include "artMap.h"

#include <sstream>

class CFusion {
public:
	CFusion();

	/**
	 * Using three given vectors, the first two will be used as the input pattern, while the last one
	 * will be used as the output pattern. The "search" operator defines if there will only be searched
	 * or also adapted/learned. Be careful, if search=false, then the robot can quickly run out of
	 * memory! So, this function classifies aor learns depending on the "search" parameter.
	 *
	 * The required input is laser and blob information. The "happinessVector" can be used for supervised
	 * or semi-supervised learning. Examples:
	 *
	 * a.) semi-supervised: a robot detects that it is coupled to a power station, the happinessVector is
	 *     set with powering versus non-powering
	 * b.) supervised: a robot sees object A and class id 0 is set in happinessVector, object B and 1 is
	 *     set by the experimenter
	 * c.) semi-supervised: a robot has two cameras, the experimenter sets two objects on both sides, the
	 *     camera location left/right is used as values in the happiness vector.
	 */
	ART_VIEW* classify(ART_ASPECT* featureA, ART_ASPECT* featureB, ART_ASPECT* rewards, bool search);

	/**
	 * Set vigilance, first laser, than blob. The happiness vector does not need a vigilance setting,
	 * because it is used as teaching signal.
	 */
	void setVigilance(float laserVigilance, float blobVigilance);

	//! Save to file
	void saveMemory(std::string filename);

	//! Load from file
	void loadMemory(std::string filename);

	//! Make happiness network available
	inline PROTOTYPE *getHappinessPrototype(int id) { return d_artReward.getPrototype(id); };
protected:
	bool isObject(std::vector<ART_TYPE>* featureA, std::vector<ART_TYPE>* featureB, std::vector<ART_TYPE>* rewards,
			bool search);

	//! Read a network from file
	void readInput(std::string filenameInput, std::vector<ART_TYPE>* inputVector);

//	void learn(int imgWidth, int imgHeight);

	Art d_artFeatureA;
	Art d_artFeatureB;
	Art d_artReward;

//	std::string d_wallInput;
//	std::string d_robotInput;
//	std::string d_blobWallInput;
//	std::string d_blobRobotInput;

	//! Final fusion map
	ArtMap d_fusionArtMap;

	//! A set of ART networks
	std::vector<Art*> d_artVectorLaserBlob;

	//! A set of ART networks
	std::vector<Art*> d_artVectorEnergy;
};

#endif /* CFUSION_H_ */
