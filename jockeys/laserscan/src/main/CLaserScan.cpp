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
		int laser_width = 640): printTime(false),
		printLaser(false),
		showDiff(false),
		image1(new CRawImage(img_width, img_height, 3)),
		image2(new CRawImage(img_width, img_height, 3)),
		sfTimer(),
		laser(robot_base, robot_type),
//		pix_buf(NULL),
		laserVec(NULL),
		laserSmallVec(NULL),
		laserSmallVecSize(0),
//		laserX(NULL),
//		laserY(NULL),
		imageWidth(img_width),
		imageHeight(img_height),
		laserResolution(laser_width),
		colorSpace(CS_RGB),
		topRowLimit(0),
		bottomRowLimit(img_height-1), // has high value because index runs from top to bottom from low to high
		threshold(20),
		diff_threshold(20),
		cameraDeviceHandler(-1) {
}

/**
 * Destructor removes everything that is not still a NULL pointer. If you delete something beforehand, make sure to set
 * its pointer to NULL.
 */
CLaserScan::~CLaserScan() {
	Stop();
}

int CLaserScan::InitCam(int imgWidth, int imgHeight, int laserResolution) {
	imageWidth = imgWidth;
	imageHeight = imgHeight;
	this->laserResolution = laserResolution;
	laserSmallVecSize = laserResolution >> 5;

	if (printLaser) {
		printf("Laser resolution set to %i\n", laserResolution);
		printf("Laser lower res set to %i\n", laserSmallVecSize);
	}
//	pix_buf = new int[imgWidth * imgHeight * 3];
	laserVec = new int[laserResolution];
	laserSmallVec = new int[laserSmallVecSize];
//	laserX = new float[laserResolution];
//	laserY = new float[laserResolution];

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
	if (printLaser)
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

	// do not save the images to disk
//	camera.saveImages(true);

	return error;
}

void CLaserScan::Stop() {
	if (printLaser)
		printf("Deallocate image-related stuff in laser\n");
	if (image1 != NULL) delete image1;
	if (image2 != NULL) delete image2;
//	if (pix_buf != NULL) delete [] pix_buf;
	if (laserVec != NULL) delete [] laserVec;
	if (laserSmallVec != NULL) delete [] laserSmallVec;
//	if (laserX != NULL) delete [] laserX;
//	if (laserY != NULL) delete [] laserY;
	laserVec = NULL; laserSmallVec = NULL;
//	laserX = NULL; laserY = NULL;
	if (printLaser)
		printf("All laser scan stuff deallocated\n");
	camera.Stop();
}

/**
 * We iterate over the width of the difference image, and then over the height (bottom and top rows are disregarded). As
 * soon as the difference between the images exceeds a certain threshold the corresponding row index becomes the value
 * in the result vector. This value "h" goes from bottomRow (high index) to topRow (low index). When the threshold is
 * not exceeded, the value becomes equal to "topRow" which is set to topRowLimit (half of the image height by default).
 * Remark: the "width" of the line can be thicker, but this method always chooses for the smallest distance.
 * Remark: no "fish-eye" kind of distortions take into account
 */
int CLaserScan::generateVector(CRawImage* laserImage, CRawImage* noLaserImage, int* vec)
{
	assert (laserImage->getwidth() == noLaserImage->getwidth());
	assert (laserImage->getheight() == noLaserImage->getheight());

	int topRow = topRowLimit;
	int bottomRow = bottomRowLimit;
	int width = laserImage->getwidth();
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

	if (printLaser)
		printf("%s: row %i-%i [width %i, height %i]\n", __func__, bottomRowLimit, topRowLimit, width, laserImage->getheight());

	// we first run over the width of the image
	for (int i = 0; i<width; ++i) {
		bool red = false;
		int h;
		// we now run over this column from the bottom row (high index) to the top row (low index)
		// because close items are more important than items far away
		for (h = bottomRow; h > topRow && !red; h--){
			pos = 3*(h*width+i);
			red = isMoreRed(*laserImage, *noLaserImage, pos);
		}
		vec[i] = laserImage->getheight()-1 - h;
		if (!red) {
			vec[i] = 0; // distance=0 if nothing was red
		}
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
			(abs(c_blue - c_red) > diff_threshold) && (abs(c_green - c_red) > diff_threshold));
}

/**
 * Create a black-white difference image. The difference in the red channel should be above
 * a certain threshold. Moreover, the difference in the green and the blue channel should
 * be small at the same time. If that wouldn't be filtered out, intensity alone would be
 * sufficient and for example the edge between floor and wall would also surpass the red
 * threshold together with the green and blue channels. Now, we are more specific searching
 * for the red laser line.
 */
CRawImage* CLaserScan::diff(CRawImage* laserImage, CRawImage* noLaserImage, bool plain)
{
	int w = laserImage->getwidth();
	int h = laserImage->getheight();
	assert (w == noLaserImage->getwidth());
	assert (h == noLaserImage->getheight());

	CRawImage *diff = new CRawImage(w, h, 3);
	unsigned char* im_diff = diff->data;

	if (plain && printLaser) printf("Will use entire diff, not only of red-channel\n");

	// x over width, y over height
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			if (plain) {
				int pos = 3*(y*w+x);
				im_diff[pos+0] = abs(laserImage->data[pos+0] - noLaserImage->data[pos+0]);
				im_diff[pos+1] = abs(laserImage->data[pos+1] - noLaserImage->data[pos+1]);
				im_diff[pos+2] = abs(laserImage->data[pos+2] - noLaserImage->data[pos+2]);
				continue;
			}
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
 * We operate on the difference image which should show us the laser lines. This function calculates per horizontal line
 * how many pixels exceeded a predefined threshold, see function diff().
 */
//unsigned char* CLaserScan::generateVector(CRawImage* diffImage, int & vec_size) {
//	int w = diffImage->getwidth();
//	int h = diffImage->getheight();
//	vec_size = h;
//	unsigned char *result = (unsigned char*)malloc(vec_size * sizeof(unsigned char));
//	unsigned char* im_diff = diffImage->data;
//	for (int y = 0; y < h; y++) {
//		int f = 0;
//		for (int x = 0; x < w; x++) {
//			int pos = 3*(y*w+x);
//			if (im_diff[pos] >= 100) f++;
//		}
//		result[y] = f;
//	}
//	return result;
//}

/**
 * Gets the data for the laser and the camera.
 */
void CLaserScan::GetData() {
	char* imagePtr;

	// Reset and start the timer (so it won't overrun)
	sfTimer.reset();
	sfTimer.start();

	// Make a picture with the laser turned off
//	camera.renewImage(image1, false); // just for the sake of it, shoot one

	laser.Off();

	if (printTime) fprintf(stdout,"Start grabbing time: %i \n", sfTimer.getTime());
//	camera.renewImage(image1, false); // don't convert, just to get the most recent one
	camera.renewImage(image1, true);
	camera.denoiseImageByCapturingAnother(image1);

	// Turn the laser on
	laser.On();

	// Take the second picture
//	camera.renewImage(image2, false);
	camera.renewImage(image2, true);
	camera.denoiseImageByCapturingAnother(image2);

	// Turn the laser off
	laser.Off();

	if (showDiff) {
		CRawImage *diffimg = diff(image2,image1, false);
		std::stringstream ss;
		ss.clear(); ss.str("");
		ss << "imagediff" << sfTimer.getTime() << ".bmp";
		diffimg->saveBmp(ss.str().c_str());

		// For debugging loop over entire image
		int nonzeros = 0;
		for (int i = 0; i < diffimg->getsize(); ++i) {
			if (diffimg->data[i] != 0) nonzeros++;
		}
		if (nonzeros < 10) {
			fprintf(stderr, "There are only %i non-zero values in diff images, cannot be good\n", nonzeros);
		}
		delete diffimg;

		diffimg = diff(image2,image1, true);
		ss.clear(); ss.str("");
		ss << "imagediff_rgb" << sfTimer.getTime() << ".bmp";
		diffimg->saveBmp(ss.str().c_str());
		delete diffimg;

		ss.clear(); ss.str("");
		ss << "image2" << sfTimer.getTime() << ".bmp";
		image2->saveBmp(ss.str().c_str());
	}

	// Compute the laser vector using the two images
	generateVector(image2,image1,laserVec);
	if (printTime) fprintf(stdout,"Laser scan detection time: %i \n",sfTimer.getTime());
//	computeScan(laserVec,laserX,laserY);
//	if (printTime) fprintf(stdout,"Laser scan computation time: %i \n",sfTimer.getTime());

	// Print laser data
	if (printLaser) {
		fprintf(stdout,"%s - Laser [%i]: ", __func__, laserResolution);
		for (int i = 0; (i < laserResolution) && printTime; i++) {
			fprintf(stdout,"%03i ",laserVec[i]);
			if ((i % 32) == 31) {
				fprintf(stdout,"\n");
				usleep(10000);
			}
		}
		fprintf(stdout,"\n");
	}
}

/**
 * Computes something magic... Probably it is mean to translate the vector in two values that indicate "x" and "y"
 * coordinates.
 */
void CLaserScan::computeScan(int* vec,float* distX,float* distY)
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

int max(int *vector, int size) {
	assert(size > 0);
	int max = vector[0];
	for (int i = 1; i < size; i++) {
		if (vector[i] > max) max = vector[i];
	}
	return max;
}

int min(int *vector, int size) {
	assert(size > 0);
	int min = 1000; // should be int-related
	for (int i = 1; i < size; i++) {
		if (vector[i] && vector[i] < min) min = vector[i]; // only non-zero mins
	}
	if (min == 1000) min = 0;
	return min;
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
 * Sub sampling of a vector. From an input vector
 *  0 1 2 3 4 5 6 7 8 9
 * And a output vector of size 4, it will add every floor(9/4)= 2nd item, to the output vector which should have size 4.
 *  0 2 4 6
 * With an output vector of size 3, it will add every 3rd item:
 *  0 3 6
 * The output vector has to be smaller than half of the input vector. Just because I am lazy in this implementation.
 */
void subsample(int *vector_in, int size_in, int *vector_out, int size_out) {
	if ((size_in>>1) <= size_out)
		fprintf(stderr, "Size in is %i while size out is %i, should be at least 2x smaller\n", size_in, size_out);
	assert((size_in>>1) > size_out);

	int spacing = floor(size_in / size_out);
	int i = 0, j= 0;
	for (; j < size_out; j++) {
		vector_out[j] = vector_in[i];
		i += spacing;
	}
}

/**
 * Fill small array
 */
void CLaserScan::Fill(int *in, int in_size, int *out, int out_size) {
	assert((in_size % out_size) == 0);
	int div_factor = in_size / out_size;
	if(printLaser) printf("%s(): From size %i to %i (factor of %i)\n", __func__, in_size, out_size, div_factor);
	for (int j = 0; j < out_size; ++j) {
		out[j] = 0;
		int sum = 0;
		for (int f = 0; f < div_factor; ++f) {
			int i = j*div_factor+f;
			if (in[i]) sum++;
			out[j] += in[i];
		}
		if (sum) {
			out[j] /= sum;
		}
		if (sum < div_factor/4) { // there must be at least say 32/4 values that are non-zero
			out[j] = 0;
		}
	}
}

/**
 * Uses the laser data to calculate the distance to an object or the wall. The value distance[i]=0 is used as not a
 * number to get rid of values that are too large.
 */
void CLaserScan::GetDistance(int &distance) {
	char max_attempts = 5;
	max_attempts = 1;
	char attempt = 0;
	distance = 0;
	int div_factor = 5;
	assert(laserResolution >> div_factor == laserSmallVecSize);
	do {
		GetData();
		attempt++;

		Fill(laserVec, laserResolution, laserSmallVec, laserSmallVecSize);

		if (!printLaser) {
			fprintf(stdout, "%s(): Smaller laser array [%i]: ", __func__, laserSmallVecSize);
			for (int i = 0; i < (laserSmallVecSize); i++)
				fprintf(stdout, "%03i ", laserSmallVec[i]);
			fprintf(stdout, "\n");
		}


		// Quick and dirty way to get distance metrics
		int min_dist = min(laserSmallVec, laserResolution >> div_factor);
		if(printLaser) printf("%s(): minimum detected non-zero value = [%i]\n", __func__, min_dist);
		assert(min_dist < imageHeight);
		if (min_dist < 10) { // too close? then cannot be accurate, the camera would not even see the line that close
			distance = 0;
		} else if (min_dist > (imageHeight-10)) {  // too far? that must be an error, camera cannot even see the laser there
			distance = 0;
		} else {
			distance = (double)20.0066 + (double)min_dist * -0.10189 + (double)min_dist*min_dist*0.00063011; //(imageHeight - max_dist); // why >> 2 // invert the distance
		}
		if(printLaser) printf("%s(): calculated distance = [%i]\n", __func__, distance);
	} while ((distance == 0) && (attempt != max_attempts));

	int laserSmallVecLength = laserResolution >> div_factor;

	// will just subsample the vector blindly
//	subsample(laserSmallVec, laserSmallVecLength, dist_vec, dist_vec_length);

	// Sleep if necessary
	//	sfTimer.tick();

//	return attempt;
}
