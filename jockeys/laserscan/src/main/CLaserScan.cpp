/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Laser scan uses camera
 * @file CLaserScan.cpp
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

#include <string.h>
#include <cassert>
#include <sstream>

#include "CLaserScan.h"
#include "CTimer.h"
#include "CCamera.h"
#include "CLaser.h"

CLaserScan::CLaserScan(RobotBase *robot_base, RobotBase::RobotType robot_type,
		int img_width = 640, int img_height = 480,
		int laser_width = 640): printTime(true),
		printLaser(true),
		showDiff(true),
		image1(new CRawImage(img_width, img_height, 3)),
		image2(new CRawImage(img_width, img_height, 3)),
		sfTimer(),
		laser(robot_base, robot_type),
		pix_buf(NULL),
		laserVec(NULL),
		laserSmallVec(NULL),
		laserX(NULL),
		laserY(NULL),
		imageWidth(img_width),
		imageHeight(img_height),
		laserResolution(laser_width),
		cameraDeviceHandler(-1) {
}

/**
 * Destructor removes everything that is not still a NULL pointer. If you delete something beforehand, make sure to set
 * its pointer to NULL.
 */
CLaserScan::~CLaserScan() {
	if (image1 != NULL) delete image1;
	if (image2 != NULL) delete image2;
	if (pix_buf != NULL) delete pix_buf;
	if (laserVec != NULL) delete laserVec;
	if (laserSmallVec != NULL) delete laserSmallVec;
	if (laserX != NULL) delete laserX;
	if (laserY != NULL) delete laserY;
}

int CLaserScan::InitCam(int imgWidth, int imgHeight, int laserResolution) {
	imageWidth = imgWidth;
	imageHeight = imgHeight;
	this->laserResolution = laserResolution;

	pix_buf = new char[imgWidth * imgHeight * 3];
	laserVec = new unsigned char[laserResolution];
	laserSmallVec = new int[laserResolution >> 4];
	laserX = new float[laserResolution];
	laserY = new float[laserResolution];

	int error;
	error = 0;
	camera.Init("/dev/video0", cameraDeviceHandler, imgWidth, imgHeight);
	if (error < 0) fprintf (stderr, "Error initialising camera\n");
	if (cameraDeviceHandler <= 0) {
		fprintf(stdout, "Camera not properly initialized.\n");
		fprintf(stdout, "Did you do \"modprobe blackfin-cam\"?\n");
		error = -1;
		return error;
	}
	fprintf(stdout,"Camera initialized\n");
	return error;
}

/**
 * Initialize the camera, the laser, and the sensor fusion module itself.
 */
int CLaserScan::Init() {
	int error;

	sfTimer.start();

	// Set loop frequency to every 2 seconds
	sfTimer.setPeriod(2000);
	error = InitCam(imageWidth, imageHeight, laserResolution);

	camera.saveImages(true);

	return error;
}

/**
 * This apparently first iterates over the width, and then over the height (where the bottom rows
 * and the top rows are disregarded). As soon as the difference between the images exceeds a
 * certain threshold this becomes the value in the vector. The value h goes from bottomRow (high
 * index) to topRow (low index). When the threshold is not exceeded the value becomes equal to
 * "topRow" which is set to topRowLimit (120 by default).
 * Remark: does not take into account that the "width" of the line can be thicker and always chooses
 * for the smallest distance.
 * Remark: does not take into account that the camera has a kind of "fish-eye".
 * It expects an "integer" on the first position...
 */
int CLaserScan::generateVector(CRawImage* laserImage, CRawImage* noLaserImage, unsigned char* vec)
{
	//	unsigned char* im1 = laserImage->data;
	//	unsigned char* im2 = noLaserImage->data;
	assert (laserImage->getwidth() == noLaserImage->getwidth());
	assert (laserImage->getheight() == noLaserImage->getheight());

	int topRow = topRowLimit;
	int bottomRow = bottomRowLimit;
	int width = laserImage->getwidth();
	//	int finalPos,pos,i=0;
	int pos;
	if (topRow < 0) topRow = 0;
	if (bottomRow > laserImage->getheight()-1) {
		fprintf(stderr, "Error, bottomRow %i is larger than height %i\n", bottomRow, laserImage->getheight() -1);
		return -1;
	}
	if (bottomRow < topRow) {
		fprintf(stderr, "Error, index should be [%i < %i] (reversed)... sorry!!\n", bottomRow, topRow);
		return -1;
	}

	//	printf("Generate vector for row %i to %i over total width %i and height %i\n", bottomRowLimit, topRowLimit, width, laserImage->height-1);
	for (int w = 0;w<width;w++){
		bool red = false;
		int h = bottomRow;
		for (h = bottomRow;h > topRow && !red;h--){
			pos = 3*(h*width+w);
			red = isMoreRed(*laserImage, *noLaserImage, pos);
		}
		vec[w] = h;
	}
	return 0;
}

/**
 * What defines red depends on the channel encoding. It seems to be the case that the
 * OpenCV images coming from YARP on the PC use channel 3 for red (BGR). The images on the
 * Surveyor robot actually use RGB (channel 1).
 */
inline bool CLaserScan::isMoreRed(CRawImage &img1, CRawImage &img2, int pos) {
	int c_red, c_green, c_blue;
	switch (colorSpace) {
	case CS_RGB:
		c_red   = abs((int)img1.data[pos]  -(int)img2.data[pos]);
		c_green = abs((int)img1.data[pos+1]-(int)img2.data[pos+1]);
		c_blue  = abs((int)img1.data[pos+2]-(int)img2.data[pos+2]);
		break;
	case CS_BGR:
		c_blue  = abs((int)img1.data[pos]  -(int)img2.data[pos]);
		c_green = abs((int)img1.data[pos+1]-(int)img2.data[pos+1]);
		c_red   = abs((int)img1.data[pos+2]-(int)img2.data[pos+2]);
		break;
	default:
		fprintf(stderr, "Unknown colorSpace, should be RGB or BGR\n");
		return false;
	}
	return ((c_red > threshold) &&
			(abs(c_blue - c_red) > diff_treshold) && (abs(c_green - c_red) > diff_treshold));
}

/**
 * Create a black-white difference image. The difference in the red channel should be above
 * a certain threshold. Moreover, the difference in the green and the blue channel should
 * be small at the same time. If that wouldn't be filtered out, intensity alone would be
 * sufficient and for example the edge between floor and wall would also surpass the red
 * threshold together with the green and blue channels. Now, we are more specific searching
 * for the red laser line.
 */
CRawImage* CLaserScan::diff(CRawImage* laserImage, CRawImage* noLaserImage)
{
	int w = laserImage->getwidth();
	int h = laserImage->getheight();
	assert (w == noLaserImage->getwidth());
	assert (h == noLaserImage->getheight());

	CRawImage *diff = new CRawImage(w, h, 3);
	unsigned char* im_diff = diff->data;

	// x over width, y over height
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			bool d;
			int pos = 3*(y*w+x);
			d = isMoreRed(*laserImage, *noLaserImage, pos);

			if (d) {
				im_diff[pos] = 200;
				im_diff[pos+1] = 200;
				im_diff[pos+2] = 200;
			} else {
				im_diff[pos] = 0;
				im_diff[pos+1] = 0;
				im_diff[pos+2] = 0;
			}
		}
	}
	return diff;
}

/**
 * We operate on the difference image which should show us the laser lines. This function
 * calculates per horizontal line how many pixels exceeded a predefined threshold, see
 * function diff().
 */
unsigned char* CLaserScan::generateVector(CRawImage* diffImage, int & vec_size) {
	int w = diffImage->getwidth();
	int h = diffImage->getheight();
	vec_size = h;
	unsigned char *result = (unsigned char*)malloc(vec_size * sizeof(unsigned char));
	unsigned char* im_diff = diffImage->data;
	for (int y = 0; y < h; y++) {
		int f = 0;
		for (int x = 0; x < w; x++) {
			int pos = 3*(y*w+x);
			if (im_diff[pos] >= 100) f++;
		}
		result[y] = f;
	}
	return result;
}

/**
 * Gets the data for the laser and the camera.
 */
void CLaserScan::GetData() {
	char* imagePtr;

	// Reset and start the timer (so it won't overrun)
	sfTimer.reset();
	sfTimer.start();

	// Make a picture with the laser turned off
	camera.renewImage(image1, false); // just for the sake of it, shoot one

	laser.Off();
	if (printTime) fprintf(stdout,"Start grabbing time: %i \n", sfTimer.getTime());
	camera.renewImage(image1, true);

	// Turn the laser on
	laser.On();

	// Take the second picture
	camera.renewImage(image2, true);

	// Turn the laser off
	laser.Off();

	if (showDiff) {
		CRawImage *diffimg = diff(image2,image1);
		std::stringstream ss; ss.clear(); ss.str("");
		ss << "imagediff" << sfTimer.getTime() << ".bmp";
		diffimg->saveBmp(ss.str().c_str());
		delete diffimg;
	}
	// Compute the laser vector using the two images
	generateVector(image2,image1,laserVec);
	if (printTime) fprintf(stdout,"Laser scan detection time: %i \n",sfTimer.getTime());
	computeScan(laserVec,laserX,laserY);
	if (printTime) fprintf(stdout,"Laser scan computation time: %i \n",sfTimer.getTime());

	// Print laser data
	if (printLaser) {
		fprintf(stdout,"%s - Laser: ", __func__);
		for (int i = 0; i< laserResolution && printTime;i++) {
			fprintf(stdout,"%03i ",laserVec[i]);
			if ((i % 32) == 31) {
				fprintf(stdout,"\n");
			}
		}
		fprintf(stdout,"\n");
	}
}

/**
 * Computes something magic...
 */
void CLaserScan::computeScan(unsigned char* vec,float* distX,float* distY)
{
	float x,y,z;
	float A[] = {1.0,0.1,0.1,0.1,1.0,0.1,0.1,0.1,1.0};
	for (int i = 0;i<laserResolution;i++){
		x = A[0]*i+A[1]*vec[i]+A[2];
		y = A[3]*i+A[4]*vec[i]+A[5];
		z = A[6]*i+A[7]*vec[i]+A[8];
		distX[i] = x/z;
		distY[i] = y/z;
	}
}

int avg(int *vector, int size) {
	int sum;
	for (int i = 0; i < size; i++) {
		sum += vector[i];
	}
	return sum / size;
}

// size > 1
int max(int *vector, int size) {
	int max = vector[0];
	for (int i = 1; i < size; i++) {
		if (vector[i] > max) max = vector[i];
	}
	return max;
}

int exceeds(int *vector, int size, int threshold) {
	int nof_excesses = 0;
	for (int i = 0; i < size; i++) {
		if (vector[i] > threshold) {
			nof_excesses++;
		}
	}
	return nof_excesses;
}

/**
 * Subsample vector, you can enable assertions, or just use sensible values.
 * There is no range checking enabled. Use an even and much larger input
 * vector and you will be fine.
 */
void subsample(int *vector_in, int size_in, int *vector_out, int size_out) {
	//	assert(!(size_in % 2));
	//	assert(size_in > size_out);
	//	assert(size_in / size_out > 1);

	int spacing = size_in / (size_out+1);
	int i = 0, j= 0;
	for (; j < size_out; j++) {
		i += spacing;
		vector_out[j] = vector_in[i];
	}
}

/**
 * Uses the laser data to calculate the distance to an object or the wall.
 */
int CLaserScan::GetDistance(int &distance, int *dist_vec, int dist_vec_length) {
	char max_attempts = 5;
	max_attempts = 1;
	char attempt = 0;
	distance = 0;
	do {
		GetData();
		attempt++;
		int div_factor = 5;

		// The default value is DEFAULT_IMG_HEIGHT/2
		for (int x = 0; x < laserResolution; ++x)
			laserVec[x] = (laserVec[x] - imageHeight/2) << 1;

		// First empty the array
		for (int i = 0; i < (laserResolution >> div_factor); i++) {
			laserSmallVec[i] = 0;
		}

		// Put it in a smaller vector (16 times smaller), for default 320 vector this is 20
		for (int x = 0; x < laserResolution; x++) {
			laserSmallVec[x >> div_factor] += laserVec[x];
		}

		// Divide by 16
		for (int i = 0; i < (laserResolution >> div_factor); i++) {
			laserSmallVec[i] >>= div_factor;
		}

		if (printLaser) {
			fprintf(stdout, "CSensorFusion::GetDistance() - Smaller laser array [%i]: ", laserResolution >> div_factor);
			for (int i = 0; i < (laserResolution >> div_factor); i++)
				fprintf(stdout, "%03i ", laserSmallVec[i]);
			fprintf(stdout, "\n");
		}

		// Quick and dirty way to get distance metrics
		int max_dist = max(laserSmallVec, laserResolution >> div_factor);

		if (max_dist < 10) {
			distance = 0;
		} else if (max_dist > (imageHeight/2-10)) {
			distance = 0;
		} else {
			distance = (imageHeight - max_dist) >> 2;
		}
	} while ((distance == 0) && (attempt != max_attempts));

	int laserSmallVecLength = laserResolution >> 4;

	// will return the values at 5, 10, and 15
	subsample(laserSmallVec, laserSmallVecLength, dist_vec, dist_vec_length);
	// Sleep if necessary
	//	sfTimer.tick();

	return attempt;
}
