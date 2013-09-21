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

bool LASER_VERBOSE = false;
bool SAVE_IMAGES_TO_DISK = false; // does not work anyway, jpeg is disabled

bool STREAM_RED_DIFF_IMAGES = true;
bool STREAM_RGB_DIFF_IMAGES = true;

bool SAVE_RED_DIFF_IMAGES_TO_DISK = false;
bool SAVE_RGB_DIFF_IMAGES_TO_DISK = false;

bool DUMMY_CAMERA = false;

CLaserScan::CLaserScan(RobotBase *robot_base, RobotBase::RobotType robot_type,
		int img_width = 640, int img_height = 480,
		int laser_width = 640): printTime(LASER_VERBOSE),
				printLaser(LASER_VERBOSE),
				showDiffRed(SAVE_RED_DIFF_IMAGES_TO_DISK),
				showDiffRGB(SAVE_RGB_DIFF_IMAGES_TO_DISK),
				streamDiffRed(STREAM_RED_DIFF_IMAGES),
				streamDiffRGB(STREAM_RGB_DIFF_IMAGES),
				//				imageManip(CS_BGR),
				imageManip(CS_RGB),
				sfTimer(),
				laser(robot_base, robot_type),
				laserVec(NULL),
				laserSmallVec(NULL),
				image1(NULL),
				image2(NULL),
				image_red_diff(NULL),
				image_rgb_diff(NULL),
				laserSmallVecSize(0),
				imageWidth(img_width),
				imageHeight(img_height),
				laserResolution(laser_width),
				topRowLimit(0),
				bottomRowLimit(img_height-1), // has high value because index runs from top to bottom from low to high
				cameraDeviceHandler(-1),
				coeff_a(20.0066),
				coeff_b(-0.10189),
				coeff_c(0.00063011) {
	// the coefficients are obtained in an easy way
	// x=[16,20,24,30,34] (cm) y=[98,156,197,231,249] (values) and p = polyfit(y,x,2) gives
	//   6.3011e-04, -1.0189e-01, 2.0066e+01

	assert(img_width > 0);
	image1 = new CRawImage(img_width, img_height, 3);
	image2 = new CRawImage(img_width, img_height, 3);
	if (showDiffRed || streamDiffRed) {
		image_red_diff = new CRawImage(img_width, img_height, 3);
	}
	if (showDiffRGB || streamDiffRGB) {
		image_rgb_diff = new CRawImage(img_width, img_height, 3);
	}

}

/**
 * Destructor removes everything that is not still a NULL pointer. If you delete something beforehand, make sure to set
 * its pointer to NULL.
 */
CLaserScan::~CLaserScan() {
	Stop();
}

/**
 * Initialize camera. In case there is already a camera initialized, this will run havoc in the system. For the laser to
 * be using the camera in that fashion it would need to request an image over shared memory or with another IPC method.
 */
int CLaserScan::InitCam(int imgWidth, int imgHeight, int laserResolution) {
	imageWidth = imgWidth;
	imageHeight = imgHeight;
	this->laserResolution = laserResolution;
	laserSmallVecSize = laserResolution >> 5;

	if (printLaser) {
		printf("%s(): Laser resolution set to %i\n", __func__, laserResolution);
		printf("%s(): Laser lower res set to %i\n", __func__, laserSmallVecSize);
	}
	laserVec = new int[laserResolution];
	laserSmallVec = new int[laserSmallVecSize];

	int error = 0;

	if (DUMMY_CAMERA) {
		camera.dummyInit("/data/blackfin/test", "image");
		return error;
	}
	camera.Init("/dev/video0", cameraDeviceHandler, imgWidth, imgHeight);
	if (error < 0) fprintf (stderr, "Error initialising camera\n");
	if (cameraDeviceHandler <= 0) {
		fprintf(stderr, "Camera not properly initialized.\n");
		fprintf(stderr, "Did you do \"modprobe blackfin-cam\"?\n");
		error = -1;
		return error;
	}
	if (printLaser)
		fprintf(stdout,"%s(): Camera initialized in the laser\n", __func__);
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

	// save images to disk, or not
	camera.saveImages(SAVE_IMAGES_TO_DISK);

	return error;
}

/**
 * Stop everything and deallocate all memory used to store the images and laserscans.
 */
void CLaserScan::Stop() {
	if (printLaser)
		printf("Deallocate image-related stuff in laser\n");
	if (image1 != NULL) delete image1;
	if (image2 != NULL) delete image2;
	if (laserVec != NULL) delete [] laserVec;
	if (laserSmallVec != NULL) delete [] laserSmallVec;
	laserVec = NULL; laserSmallVec = NULL;
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
 * Remark: no "fish-eye" kind of distortions take into account.
 *
 * @param laserImage         the image from the camera with the laser turned on
 * @param noLaserImage       the image from the camera with the laser turned off
 * @param vec                the vector with the (vertical) y-position of the laser line, vec[i]=0 if nothing detected
 * @return                   success (0) or failure (<0)
 */
int CLaserScan::generateVector(CRawImage* laserImage, CRawImage* noLaserImage, int* vec)
{
	assert (laserImage->getwidth() == noLaserImage->getwidth());
	assert (laserImage->getheight() == noLaserImage->getheight());

	int threshold = 20;
	int diff_threshold = 15;

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
		printf("%s(): row %i-%i [width %i, height %i]\n", __func__, bottomRowLimit, topRowLimit, width, laserImage->getheight());

	// we first run over the width of the image
	for (int i = 0; i<width; ++i) {

		bool red = false;
		int h;
		// we now run over this column from the bottom row (high index) to the top row (low index)
		// because close items are more important than items far away
		for (h = bottomRow; h > topRow && !red; h--){
			pos = 3*(h*width+i);
			// if we detect red as red enough, we set the flag
			red = imageManip.isMoreRed(*laserImage, *noLaserImage, pos, threshold, diff_threshold);
		}
		// broken out of the loop, so either red detected, or h became topRow
		vec[i] = laserImage->getheight()-1 - h;
		// just set to 0 if no red detected
		if (!red) {
			vec[i] = 0; // distance=0 if nothing was red
		}
	}
	return 0;
}



int CLaserScan::generateVector(CRawImage* laserImage, CRawImage* noLaserImage, std::vector<int> & vec) {

	assert (laserImage->getwidth() == noLaserImage->getwidth());
	assert (laserImage->getheight() == noLaserImage->getheight());

	// clear the vector first
	vec.clear();
	int margin_left = 120;
	int margin_right = 220; // 640 = 320 + 320 = 120 + 200 + 100 + 220;
	int threshold = 20;
	int diff_threshold = 15;
	for (int j = 0; j < laserImage->getheight(); j++) {
		int p = 0;
		for (int i = margin_left; i < laserImage->getwidth() - margin_right; i++) {
			int pos = 3*(j*laserImage->getwidth()+i);
			bool red = imageManip.isMoreRed(*laserImage, *noLaserImage, pos, threshold, diff_threshold);
			if (red) {
				p = i;
				break;
			}
		}
		vec.push_back(p);
	}
	if (printLaser) {
		fprintf(stdout, "%s(): Laser array [%li]: \n", __func__, vec.size());
		for (int i = 0; i < vec.size(); i++) {
			fprintf(stdout, "%03i ", vec[i]);
			if ((i % 32) == 31) {
				fprintf(stdout,"\n");
			}
		}
		fprintf(stdout, "\n");
	}

	return 0;
}

/**
 * Return the length of the detected red line. Assumes that this is one line only. And estimates the distance to that
 * line. If the thing portrayed on is too close, it will not be seen by the camera and the red-line will be portrayed
 * till the bottom of the visual field of the camera.
 */
void CLaserScan::estimateParameters(std::vector<int> & vec, int & length, int & distance, int &start, int &end) {
	// count the number of successive non-zeros for now (three zeros breaks the line)
	length = 0;
	distance = 0;
#ifdef STRATEGY_CONSECUTIVE
	int gap = 0;
	for (int i = 0; i < vec.size() && gap < 4; i++) {
		if (vec[i]) {
			length++;
			gap=0;
		} else if (length) {
			gap++;
		}
	}
#elif STRATEGY_SIMPLE
	for (int i = 0; i < vec.size(); i++) {
		if (vec[i]) {
			length++;
		} else if (!length) {
			distance++;
		}
	}
#else
	// just add all the non-zero entries
	int avg = 0; int cnt = 0; start = 0; end = 0;
	for (int i = 0; i < vec.size(); i++) {
		if (vec[i]) {
			avg += vec[i];
			length++;
		}
	}
	for (int i = 3; i < vec.size(); i++) {
		if (vec[i] && vec[i-1] && vec[i-2]) {
			start = i-2;
			break;
		}
	}
	for (int i = vec.size()-3; i > 0; i--) {
		if (vec[i] && vec[i+1] && vec[i+2]) {
			end = i+2;
			break;
		}
	}


	if (length)
		distance = avg / length;
#endif
}

/**
 * Get line out of the image using the Hough transform.
 *
 * @param image              Image with differences between laser turned on and off
 * @param alpha              Line angle
 * @param d                  Distance of line to origin
 */
void CLaserScan::getLine(CRawImage *image, double & alpha, double & d) {
	std::vector<DecPoint*> points;
	points.clear();
	for (int i = 0; i < image->getwidth(); ++i) {
		for (int j = 0; j < image->getheight(); ++j) {
			Pixel p = image->getPixel(i,j);
			if (p.r > 50) {
				points.push_back(new DecPoint(i,j));
			}
		}
	}
	std::cout << "Created a vector with " << points.size() << " points " << std::endl;

	hough.addPoints(points);

	int hough_steps = 40;
	std::cout << "Use " << hough_steps << " hough steps" << std::endl;

	std::cout << "Hough transform status ";
	for (int t = 0; t < hough_steps; t++) {
		std::cout << '.';
		hough.doTransform();
	}
	std::cout << std::endl;

	hough.getLine(d, alpha);

	hough.clear();

	std::cout << "Recognized line of angle " << alpha << " and distance " << d << std::endl;
}

/**
 * Gets the data for the laser and the camera.
 */
void CLaserScan::GetData() {
	if (printLaser) printf("%s(): Main function to get camera information with the laser turned off vs on\n", __func__);
	char* imagePtr;

	// Reset and start the timer (so it won't overrun)
	sfTimer.reset();
	sfTimer.start();

	laser.Off();

	if (printTime) fprintf(stdout,"Start grabbing time: %ims\n", sfTimer.getTime());
	camera.renewImage(image1, true);
	//	camera.denoiseImageByCapturingAnother(image1);

	assert (imageManip.CheckIntegrity(image1) );

	// Turn the laser on
	laser.On();

	// Take the second picture
	camera.renewImage(image2, true);
	//	camera.denoiseImageByCapturingAnother(image2);

	// Turn the laser off
	laser.Off();

	int time = sfTimer.getTime(); // make sure, the time is from capturing... more or less, not from writing
	//	time = 0; // for debugging purposes, or else the thing is called differently all the time
	if (printTime) fprintf(stdout,"Finished grabbing at time: %ims\n", time);

	if (showDiffRed || streamDiffRed) {
		// save the image that only diffs the red channel
		imageManip.diff_red(image2,image1,image_red_diff);
		if (printTime) fprintf(stdout,"Done red-diffing image: %ims\n", sfTimer.getTime());

		if (showDiffRed) {
			imageManip.CheckIntegrity(image_red_diff);
			std::stringstream ss; ss.clear(); ss.str("");
			ss << "red_imagediff" << time << ".bmp";
			image_red_diff->saveBmp(ss.str().c_str());
		}
	}

	if (showDiffRGB || streamDiffRGB) {
		// save the image that diffs all channels
		imageManip.diff_rgb(image2,image1,image_rgb_diff);
		if (printTime) fprintf(stdout,"Done rgb-diffing image: %ims\n", sfTimer.getTime());

		if (showDiffRGB) {
			imageManip.CheckIntegrity(image_rgb_diff);
			std::stringstream ss; ss.clear(); ss.str("");
			ss << "rgb_imagediff" << time << ".bmp";
			image_rgb_diff->saveBmp(ss.str().c_str());
		}
	}

	if (showDiffRed || showDiffRGB) {
		// also store the raw images themselves for comparison
		std::stringstream ss; ss.clear(); ss.str("");
		ss << "raw1_image" << time << ".bmp";
		image1->saveBmp(ss.str().c_str());

		// also store the raw images themselves for comparison
		ss.clear(); ss.str("");
		ss << "raw2_image" << time << ".bmp";
		image2->saveBmp(ss.str().c_str());
	}

#ifdef CALCULATE_HOUGH
	// Compute the orientation of the line and its distance to the origin
	double alpha = 0, d = 0;
	getLine(image_red_diff,alpha,d);

	if (printTime) fprintf(stdout,"%s(): Laser scan detection time: %ims\n",__func__,sfTimer.getTime());
#endif

	// Compute the laser vector using the two images
	//generateVector(image2,image1,laserVec);
	//
	//	// Print laser data
	//	if (printLaser) {
	//		fprintf(stdout,"%s(): Laser [%i]: \n", __func__, laserResolution);
	//		for (int i = 0; (i < laserResolution) && printTime; i++) {
	//			fprintf(stdout,"%03i ",laserVec[i]);
	//			if ((i % 32) == 31) {
	//				fprintf(stdout,"\n");
	//				usleep(10000);
	//			}
	//		}
	//		fprintf(stdout,"\n");
	//	}
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
 * Compress values so they fit in a small array. The value "0" will be used not-a-number. It will not be taken into
 * account calculating the average for the outgoing values.
 */
void CLaserScan::Fill(int *in, int in_size, int *out, int out_size) {
	assert (in != NULL);
	assert (out != NULL);
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
 * Get recognized object.
 */
void CLaserScan::GetRecognizedObject(ObjectType &object_type, int & distance) {
	// get the two camera images and calculate the difference
	GetData();

	generateVector(image2,image1,laserVector);

	distance = 0;
	int length = 0, start = 0, end = 0;
	estimateParameters(laserVector, length, distance, start, end);

//	if (printLaser) {
		std::cout << "Laser parameters: length=" << length << ", distance=" << distance << ", start=" << start << ", end=" << end << std::endl;
//	}

	// used octave to fit the stuff, first array is the distance in cm, second array are the values from the laserscan
	// x=[7,10,13,16,19,22,25,28,31,34,37];
	// y=[151,204,234,250,265,276,281,287,292,298,300];
	// p = polyfit(y,x,2)
	// 1.9176e-03  -6.8800e-01   6.8157e+01
	double coeff_a = 68.157;
	double coeff_b = -0.688;
	double coeff_c = 0.0019176;
	distance = coeff_a + (double)distance * coeff_b + (double)distance*distance*coeff_c;

	/*
	 * small step (very high last value):
	 *   length distance start end
	 *  -  99 228 381 479
	 *  - 120 258  32 479
	 *  -  99 229 381 479
	 *  -  96 229 383 478
	 *  -  98 228 381 478
	 *  - 109 302 231 339
	 *  - 107 302 231 338
	 *  - 124 295 250 373
	 *  - 123 295 251 375
	 *  -  94 313 205 298
	 *  - 139 247 341 479
	 *  - 140 247 340 477
	 *
	 * large step (quite a low third value):
	 *  - 231 271 216 446
	 *  - 194 208 285 478
	 *  - 195 208 295 479
	 *  - 168 298 187 352
	 *  - 179 271 178 477
	 *  - 153 299 178 330
	 * length=242, distance=233, start=238, end=479
	 * length=207, distance=273, start=183, end=389
	 *
	 *  wall (characterized by very low value for 3):
	 *  - 242 295 105 347
	 *  - 337 294  11 348
	 *  - 312 294   1 348
	 *  - 161 320  61 268
	 *
	 *  length=181, distance=301, start=123, end=310
	 *  length=304, distance=300, start=1, end=318
	 */

	printf("Laser detected: \n");
	if (distance > 40) {
		printf("* nothing for now\n");
		object_type = O_NOTHING;
		return;
	} else if (length > 260) { // larger lines are definitely a wall
		printf("* a wall because the vertical structure is very long\n");
		object_type = O_WALL;
		return;
	} else if (length > 180 && (start < 150)) { // start very low means that the line goes up very high, must be a wall
		printf("* a wall because there is something far away\n");
		object_type = O_WALL;
		return;
	} else if (length > 50 && (start < 100)) { // if start is very low, it is definitely a wall, even if length is small
		printf("* a wall because there is something very far away\n");
		object_type = O_WALL;
		return;
	} else if (length < 100) {
		printf("* a small step because there is a very tiny line\n");
		object_type = O_SMALL_STEP;
		return;
	} else if ((length < 150) && (start > 250)) {
		printf("* a small step because there is a tiny line and it is nearby\n");
		object_type = O_SMALL_STEP;
		return;
	} else if (length < 260) { //	 && (distance/length < 2.0) ) {
		printf("* a large step because there is a nice line\n");
		object_type = O_LARGE_STEP;
		return;
	} else {
		printf("* something, but has to decide\n");
		object_type = O_SOMETHING;
		return;
	}
}

/**
 * Uses the laser data to calculate the distance to an object or the wall. The value distance[i]=0 is used as not a
 * number to get rid of values that are too large.
 *
 * @param distance           distance to be set, will be set to 0 if there is no laser ray detected
 */
void CLaserScan::GetDistance(int &distance) {
	distance = 0;
	int div_factor = 5;
	assert(laserResolution >> div_factor == laserSmallVecSize);

	GetData();

	generateVector(image2,image1,laserVector);
	int length = 0, start = 0, end = 0;
	estimateParameters(laserVector, length, distance, start, end);
	//	if (printLaser) {
	fprintf(stdout,"%s(): Line length: %i pixels\n", __func__, length);
	//		fprintf(stdout,"%s(): Distance to line: %i cm\n", __func__, distance);
	//	}

	//		x=[7,10,13,16,19,22,25,28,31,34,37];
	//		y=[151,204,234,250,265,276,281,287,292,298,300];
	//		p = polyfit(y,x,2)
	//
	//		   1.9176e-03  -6.8800e-01   6.8157e+01
	double coeff_a = 68.157;
	double coeff_b = -0.688;
	double coeff_c = 0.0019176;
	distance = coeff_a + (double)distance * coeff_b + (double)distance*distance*coeff_c;


#ifdef CALC_DISTANCE
	Fill(laserVec, laserResolution, laserSmallVec, laserSmallVecSize);

	if (printLaser) {
		fprintf(stdout, "%s(): Smaller laser array [%i]: ", __func__, laserSmallVecSize);
		for (int i = 0; i < (laserSmallVecSize); i++)
			fprintf(stdout, "%03i ", laserSmallVec[i]);
		fprintf(stdout, "\n");
	}

	// Quick and dirty way to get distance metrics
	int min_dist = min(laserSmallVec, laserResolution >> div_factor);
	if(printLaser) printf("%s(): minimum detected non-zero value = [%i]\n", __func__, min_dist);
	assert(min_dist < imageHeight);
	if (min_dist < 10) {
		// too close? then cannot be accurate, the camera would not even see the line that close
		distance = 0;
	} else if (min_dist > (imageHeight-10)) {
		// too far? that must be an error, camera cannot even see the laser there
		distance = 0;
	} else {
		// the following values are calculated from the laser scan on a wall at different distances
		distance = coeff_a + (double)min_dist * coeff_b + (double)min_dist*min_dist*coeff_c;
	}
	if(printLaser) printf("%s(): calculated distance = [%i]\n", __func__, distance);
#endif
}
