/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Laser scan uses camera
 * @file CLaserScan.h
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
 * @date      Jan 18, 2011
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CSENSORFUSION_H_
#define CSENSORFUSION_H_

/***********************************************************************************************************************
 * Jockey framework includes
 **********************************************************************************************************************/

#include "CRawImage.h"
#include "CImageManip.h"
#include "CTimer.h"
#include "CLaser.h"
#include "CCamera.h"

#include <Hough.h>
#include <DetectLineModuleExt.h>

/***********************************************************************************************************************
 * Nice feature, á la YARP middleware
 **********************************************************************************************************************/

#define VOCAB(a,b,c,d) ((((int)(d))<<24)+(((int)(c))<<16)+(((int)(b))<<8)+((int)(a)))
#define VOCAB4(a,b,c,d) VOCAB((a),(b),(c),(d))
#define VOCAB3(a,b,c) VOCAB((a),(b),(c),(0))
#define VOCAB2(a,b) VOCAB((a),(b),(0),(0))
#define VOCAB1(a) VOCAB((a),(0),(0),(0))

/***********************************************************************************************************************
 * Interface
 **********************************************************************************************************************/

//! Define the different object types as integers, so they can used in a switch statements and - on the other hand - be
//! understood immediately.
#define O_SMALL_STEP         VOCAB4('s','t','e','p')
#define O_LARGE_STEP         VOCAB4('S','T','E','P')
#define O_WALL               VOCAB4('w','a','l','l')
#define O_NOTHING            VOCAB4('n','a','d','a')
#define O_SOMETHING          VOCAB4('s','o','m','e')

//! They are of the type "int"
typedef int ObjectType ;

/**
 * Uses data from laser and camera for e.g. distance information
 */
class CLaserScan {
public:
	//! Construct laserscan and all corresponding objects like camera and laser data array
	CLaserScan(RobotBase *robot_base, RobotBase::RobotType robot_type, int img_width, int img_height, int laser_width);

	//! Destroy laserscan and all objects
	~CLaserScan();

	//! All the necessary initialisation, needs to be called before anything
	int Init();

	//! Stop everything
	void Stop();

	//! Get the distance to an object using laser and camera data
	void GetDistance(int &distance);

	//! Set the refresh rate of the module (will make the distance less accurate of course)
	void SetRefreshRate(int rate);

	//! Fill small array from big one
	void Fill(int *in, int in_size, int *out, int out_size);

	//! Setter for limits, top < bottom... (awkward, yes)
	inline void setLimits(int top, int bottom) { topRowLimit = top; bottomRowLimit = bottom; }

	CRawImage *getRGBDiffImg() {
		assert (image_rgb_diff != NULL);
		return image_rgb_diff;
	}

	CRawImage *getRedDiffImg() {
		assert (image_red_diff != NULL);
		return image_red_diff;
	}

	CRawImage *getImg1() {
		assert (image1 != NULL);
		return image1;
	}

	CRawImage *getImg2() {
		assert (image2 != NULL);
		return image2;
	}

	inline CCamera & GetCamera() { return camera; }

	//! Get recognized object type
	void GetRecognizedObject(ObjectType &object_type, int & distance);
protected:
	//! Get the laser scan
	void GetData();

	//! Initialize the camera
	int InitCam(int imgWidth, int imgHeight, int laserResolution);

	//! Generate laser vector using two cameras, one with laser on, one with laser off
	int generateVector(CRawImage* laserImage, CRawImage* noLaserImage, int* vec);

	int generateVector(CRawImage* laserImage, CRawImage* noLaserImage, std::vector<int> & vec);

	//! Get line from an image
	void getLine(CRawImage *image, double & alpha, double & d);

	void estimateParameters(std::vector<int> & vec, int & length, int & distance, int &start, int &end);

private:
	//! The internal structure for the Hough transform
	dobots::Hough<DecPoint> hough;

	bool printTime;

	bool printLaser;

	bool showDiffRed;

	bool showDiffRGB;

	bool streamDiffRed;

	bool streamDiffRGB;

	CRawImage *image1,*image2,*image_red_diff,*image_rgb_diff;

	CTimer sfTimer;

	CCamera camera;

	CLaser laser;

	CImageManip imageManip;

	int cameraDeviceHandler;

	std::vector<int> laserVector;

	int *laserVec;
	int *laserSmallVec;
	int laserSmallVecSize;

	int imageWidth;
	int imageHeight;
	int laserResolution;

	int topRowLimit;
	int bottomRowLimit;

	//! Coefficients used to calculate from vertical line height to distance, this depends on the laser and camera
	//! placement
	double coeff_a;
	double coeff_b;
	double coeff_c;

};

#endif /* CSENSORFUSION_H_ */
