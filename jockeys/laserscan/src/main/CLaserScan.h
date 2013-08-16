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
 * Interface
 **********************************************************************************************************************/

/**
 * Uses data from laser and camera for e.g. distance information
 */
class CLaserScan {
public:
	CLaserScan(RobotBase *robot_base, RobotBase::RobotType robot_type, int img_width, int img_height, int laser_width);

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

	CRawImage *getRGBDiffImg() { return image_rgb_diff; }

	CRawImage *getRedDiffImg() { return image_red_diff; }

	CRawImage *getImg1() { return image1; }

	CRawImage *getImg2() { return image2; }

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

//	int estimateLength(std::vector<int> & vec);
	void estimateParameters(std::vector<int> & vec, int & length, int & distance);

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
