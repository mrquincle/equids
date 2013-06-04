/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Sensor fusion functionality using Adaptive Resonance Theory
 * @file CFusion.cpp
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
 * @date      Jul 30, 2012
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Testing
 */

#include "CFusion.h"

using namespace std;

/* This Constructor has predefined values
 * these values have been determined empirically
 */
CFusion::CFusion(): d_artFeatureA(false, true, true),
		d_artFeatureB(false, true, true),
		d_artReward(false, false, true),
		d_artVectorLaserBlob(3),
		d_fusionArtMap(&d_artVectorLaserBlob)
{
	d_artFeatureA.setVigilance(0.97);

	d_artFeatureB.setVigilance(0.95);
	d_artFeatureB.setNetworkReliability(0.9);

	/***** Energy *****/
	d_artReward.setVigilance(0.99);
	d_artReward.setNetworkReliability(0.9);

	/***** Fusion *****/
	d_artVectorLaserBlob[0] = &d_artFeatureA;
	d_artVectorLaserBlob[1] = &d_artFeatureB;
	d_artVectorLaserBlob[2]	= &d_artReward;

	/***** Learn input *****/
//	d_wallInput = "wall.vec";
//	d_robotInput = "robot.vec";
//	d_blobWallInput = "blobWall.vec";
//	d_blobRobotInput = "blobRobot.vec";
}

void CFusion::setVigilance(float featureA_vigilance, float featureB_vigilance)
{
	d_artFeatureA.setVigilance(featureA_vigilance);
	d_artFeatureB.setVigilance(featureB_vigilance);
}

/**
 * Classification of an object using two types of sensor data available. The "happinessVector"
 * can be used for supervised or unsupervised learning. For example, if it is somehow
 * possible to have the robot detect that it is coupled to a power outlet, this
 * information can be directly fed into the happinessVector. In the case of supervised
 * learning it is possible to enter a class id like "0" or "1" for different observable
 * objects.
 * A very simple way of "supervised" learning would be to use the camera index as the
 * happiness value.
 */
ART_DISTRIBUTED_CLASSES* CFusion::classify(ART_ASPECT* featureA,
		ART_ASPECT* featureB, ART_DISTRIBUTED_CLASS* rewards, bool search)
{
	d_artFeatureB.setTestMatch(search);
	d_artFeatureA.setTestMatch(search);
	//d_artHappyness.setTestMatch(search);
	ART_VIEW inputVector(0);

	// We push all supervised features from different modalities on the ARTMAP stack
	inputVector.push_back(featureA);
	inputVector.push_back(featureB);
	inputVector.push_back(rewards);

	// TODO: add search option
	ART_DISTRIBUTED_CLASSES* output;

	if(search)
	{
		// set vigilance to 0 (?)
		float featureB_vig = d_artFeatureB.getVigilance();
		float featureA_vig = d_artFeatureA.getVigilance();
		// set vigilance to 0.8
		d_artFeatureB.setVigilance(0.8);
		d_artFeatureA.setVigilance(0.8);
		int found = 0;
		vector<int> winnCount(0);

		// we only have one "view" for ARTMAP (at t=now).
		ART_VIEWS multipleInputVector(0);
		multipleInputVector.push_back(&inputVector);

		// returns a vector of classes (each represented in a distributed manner)
		ART_DISTRIBUTED_CLASS& classes = *d_fusionArtMap.distMapNodeClassification(
				&multipleInputVector, &found, &winnCount);

		// create a vector with only the first three classes, a bit redundant...
		output = new ART_DISTRIBUTED_CLASSES(0);

		ART_DISTRIBUTED_CLASS* in0 =  new ART_DISTRIBUTED_CLASS(0);
		ART_DISTRIBUTED_CLASS* in1 =  new ART_DISTRIBUTED_CLASS(0);
		ART_DISTRIBUTED_CLASS* in2 =  new ART_DISTRIBUTED_CLASS(0);

		in0->push_back(classes[0]);
		in1->push_back(classes[1]);
		in2->push_back(classes[2]);

		output->push_back(in0);
		output->push_back(in1);
		output->push_back(in2);
		// set vigilance back, 0?
		d_artFeatureB.setVigilance(featureB_vig);
		d_artFeatureA.setVigilance(featureA_vig);
	}
	else {
		// actual classification
		output = d_fusionArtMap.classify(inputVector);
	}
	//d_artMapLaserBlob.printArtMap();
	return output;
}

bool CFusion::isObject(vector<ART_TYPE>* featureA, vector<ART_TYPE>* featureB,
		vector<ART_TYPE>* rewards, bool search)
{
	d_artFeatureB.setTestMatch(search);
	d_artFeatureA.setTestMatch(search);
	d_artReward.setTestMatch(search);
	vector<vector<ART_TYPE>*> inputVector(0);

	inputVector.push_back(featureA);
	inputVector.push_back(featureB);
	inputVector.push_back(rewards);

	// TODO: add search option
	vector<vector<ART_TYPE>*>* output;

	if(search)
	{
		float featureB_vig = d_artFeatureB.getVigilance();
		float featureA_vig = d_artFeatureA.getVigilance();
		d_artFeatureB.setVigilance(0.6);
		d_artFeatureA.setVigilance(0.6);
		int found  = 0;
		vector<int> winnCount(0);
		vector<vector<vector<ART_TYPE>*>*> multipleInputVector(0);
		multipleInputVector.push_back(&inputVector);
		vector<ART_TYPE>* classes = d_fusionArtMap.distMapNodeClassification(&multipleInputVector, &found, &winnCount);
		output = new vector<vector<ART_TYPE>*>(0);
		vector<ART_TYPE>* in0 =  new vector<ART_TYPE>(0);
		in0->push_back(classes->at(0));
		vector<ART_TYPE>* in1 =  new vector<ART_TYPE>(0);
		in1->push_back(classes->at(1));
		vector<ART_TYPE>* in2 =  new vector<ART_TYPE>(0);
		in2->push_back(classes->at(2));
		output->push_back(in0);
		output->push_back(in1);
		output->push_back(in2);
		d_artFeatureB.setVigilance(featureB_vig);
		d_artFeatureA.setVigilance(featureA_vig);
	}
	else
		output = d_fusionArtMap.classify(inputVector);
	bool isObject = d_artReward.getPrototype((int)output->at(2)->at(0))->at(0)==1;
	for (int x = 0; x < output->size(); ++x)
		delete (*output)[x];
	delete output;
	return isObject;
}

/**
 * Read input and store in given vector.
 */
void CFusion::readInput(string filenameInput, vector<ART_TYPE>* inputVector)
{
	printf("Reading inputfile:%s\n",filenameInput.c_str());
	ifstream inputFile(filenameInput.c_str(), ios::in);
	if(!inputFile)
		printf( "Cannot open input file.\n");
	else
	{
		while(!inputFile.eof())
		{
			int value;
			inputFile >> value;
			if(inputFile.eof())
				break;
			inputVector->push_back((ART_TYPE)value);
		}

	}
}

/*
void CFusion::learn(int imgWidth, int imgHeight)
{
	bool search = false;
	vector<ART_TYPE>* wallVector 		= new vector<ART_TYPE>(0);
	vector<ART_TYPE>* blobWallVector 	= new vector<ART_TYPE>(0);
	vector<ART_TYPE>* happynessVector	= new vector<ART_TYPE>(0);
	readInput(d_wallInput, wallVector);
	for (int x = 0; x < wallVector->size(); ++x)
		(*wallVector)[x] = ((float)((*wallVector)[x])-120)/135.0f;
	fprintf(stdout,"Wallvector size:%i\n",wallVector->size());
	readInput(d_blobWallInput, blobWallVector);
	(*blobWallVector)[0] = ((float)(*blobWallVector)[0])/(float)(imgWidth*imgHeight);
	(*blobWallVector)[1] = ((float)(*blobWallVector)[1])/(float)(imgWidth);
	(*blobWallVector)[2] = ((float)(*blobWallVector)[2])/(float)imgHeight;
	fprintf(stdout,"BlobWallvector size:%i\n",blobWallVector->size());
	happynessVector->push_back(0);
	vector<vector<ART_TYPE>*>* output = classify(wallVector,  blobWallVector,  happynessVector,  search);
	printf("Is Object: %s\n", d_artReward.getPrototype((int)output->at(2)->at(0))->at(0)==0?"no":"yes");
	for (int x = 0; x < output->size(); ++x)
		delete (*output)[x];
	delete output;

	vector<ART_TYPE>* robotVector 		= new vector<ART_TYPE>(0);
	vector<ART_TYPE>* blobRobotVector 	= new vector<ART_TYPE>(0);
	readInput(d_robotInput, robotVector);
	for (int x = 0; x < robotVector->size(); ++x)
		(*robotVector)[x] = ((float)((*robotVector)[x])-120)/135.0f;
	fprintf(stdout,"RobotVector size:%i\n",robotVector->size());
	readInput(d_blobRobotInput, blobRobotVector);
	(*blobRobotVector)[0] = ((float)(*blobRobotVector)[0])/(float)(imgWidth*imgHeight);
	(*blobRobotVector)[1] = ((float)(*blobRobotVector)[1])/(float)(imgWidth);
	(*blobRobotVector)[2] = ((float)(*blobRobotVector)[2])/(float)imgHeight;
	(*happynessVector)[0] = 1;

	output = classify(robotVector,  blobRobotVector,  happynessVector,  search);
	printf("Is Object: %s\n", d_artReward.getPrototype((int)output->at(2)->at(0))->at(0)==0?"no":"yes");
	for (int x = 0; x < output->size(); ++x)
		delete (*output)[x];
	delete output;


	happynessVector = NULL;

	output = classify(robotVector,  blobRobotVector,  happynessVector,  true);
	//printf("Test Is Object: %s\n", d_artHappyness.getPrototype((int)output->at(2)->at(0))->at(0)==0?"no":"yes");
	for (int x = 0; x < output->size(); ++x)
		delete (*output)[x];
	delete output;


	delete blobWallVector;
	delete blobRobotVector;
	delete happynessVector;
	delete wallVector;
	delete robotVector;
}*/

/**
 * This routine saveMemory needs to be called if you are planning to use the learnt ART networks
 * for further object recognition. Remember to remove the .art files for a pristine run. If they
 * are present, they will be automatically loaded into memory.
 */
void CFusion::saveMemory(std::string fileName)
{
	stringstream ss;
	ss << fileName;
	ss << "-FeatureB.art";
	d_artFeatureB.saveArtNetwork(ss.str());

	stringstream ss1;
	ss1 << fileName;
	ss1 << "-FeatureA.art";
	d_artFeatureA.saveArtNetwork(ss1.str());

	stringstream ss2;
	ss2 << fileName;
	ss2 << "-Rewards.art";
	d_artReward.saveArtNetwork(ss2.str());

	stringstream ss3;
	ss3 << fileName;
	ss3 << "-FusionMap.art";
	d_fusionArtMap.saveArtMap(ss3.str());
}

/**
 * Store several ART networks to the given file. There are two networks for both of the features, one network
 * for the rewards or supervised signals, and one ARTMAP that combines all together.
 */
void CFusion::loadMemory(std::string fileName)
{
	stringstream ss; ss.clear(); ss.str("");
	ss << fileName;
	ss << "-FeatureB.art";
	d_artFeatureB.loadArtNetWork(ss.str());

	ss.clear(); ss.str("");
	ss << fileName;
	ss << "-FeatureA.art";
	d_artFeatureA.loadArtNetWork(ss.str());

	ss.clear(); ss.str("");
	ss << fileName;
	ss << "-Rewards.art";
	d_artReward.loadArtNetWork(ss.str());

	ss.clear(); ss.str("");
	ss << fileName;
	ss << "-FusionMap.art";
	d_fusionArtMap.loadArtMap(ss.str());
}
