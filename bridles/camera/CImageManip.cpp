/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CImageManip.cpp
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

#include <CImageManip.h>

#include <cassert>

/**
 * Set besides the color space also two little arrays to go back and forth from channel index to color.
 */
void CImageManip::setColorSpace(ColorSpace colorSpace) {
	this->colorSpace = colorSpace;
	switch(colorSpace) {
	case CS_RGB: {
		channel_color[0] = C_RED;
		channel_color[1] = C_GREEN;
		channel_color[2] = C_BLUE;
		channel_index[C_RED] = 0;
		channel_index[C_GREEN] = 1;
		channel_index[C_BLUE] = 2;
		break;
	}
	case CS_BGR:
		channel_color[0] = C_BLUE;
		channel_color[1] = C_GREEN;
		channel_color[2] = C_RED;
		channel_index[C_BLUE] = 0;
		channel_index[C_GREEN] = 1;
		channel_index[C_RED] = 2;
		break;
	}
}

/**
 * Return true if pixel at position @pos exceeds the value for a particular color channel compared to the second image.
 * @param img1               First image (which should have the greater value on true)
 * @param img2               Second image
 * @param pos                Position of the pixel (1-dimensional)
 * @param color              Color to compare
 * @return                   boolean true or false
 */
bool CImageManip::greater_than(CRawImage &img1, CRawImage &img2, int pos, Color color) {
	return ((int)img1.data[pos+channel_index[color]] > (int)img2.data[pos+channel_index[color]]);
}

/**
 * What defines red depends on the channel encoding. It seems to be the case that the OpenCV images coming from YARP on
 * the PC use channel 3 for red (BGR). The images on the Surveyor robot actually use RGB (channel 1).
 * @param img1               First image, with laserscan on it
 * @param img2               Second image
 * @param pos                Position of the pixel
 * @param threshold          The threshold of the difference in the red channel comparing both images at this pixel
 * @param diff_threshold     The threshold used to see how much this difference is different from the differences in the
 *                           green and blue channels.
 */
bool CImageManip::isMoreRed(CRawImage &img1, CRawImage &img2, int pos, int threshold, int diff_threshold) {
	int c_red, c_green, c_blue;
	switch (colorSpace) {
	case CS_RGB:
		c_red   = (int)img1.data[pos+0]-(int)img2.data[pos+0];
		c_green = (int)img1.data[pos+1]-(int)img2.data[pos+1];
		c_blue  = (int)img1.data[pos+2]-(int)img2.data[pos+2];
		break;
	case CS_BGR:
		c_blue   = (int)img1.data[pos+0]-(int)img2.data[pos+0];
		c_green = (int)img1.data[pos+1]-(int)img2.data[pos+1];
		c_red  = (int)img1.data[pos+2]-(int)img2.data[pos+2];
		break;
	default:
		fprintf(stderr, "Unknown colorSpace, should be RGB or BGR\n");
		return false;
	}
	bool enough_red = (c_red > threshold);
	if (!enough_red) return false;

	// say red goes up, but blue and green goes down a lot

	bool too_much_green = (abs(c_green) > diff_threshold);

	bool too_much_blue = (abs(c_blue) > diff_threshold);

	bool different_enough = (abs(abs(c_blue) - abs(c_red)) > diff_threshold) && (abs(abs(c_green) - abs(c_red)) > diff_threshold);
	//	if (c_red > 0) {
	//		if (c_blue )
	//	}
	//return		(abs(abs(c_blue) - abs(c_red)) > diff_threshold) && (abs(abs(c_green) - abs(c_red)) > diff_threshold);
	return enough_red && !too_much_green && !too_much_blue && different_enough;
	//	return true;
}

/**
 * Create a difference image with still three channels. The difference in the red channel of the laser and no-laser
 * image, should be above a certain threshold and then the difference is written to the output.
 *
 * Moreover, the difference in the green and the blue channel should be small at the same time. If that wouldn't be
 * filtered out, intensity alone would be sufficient and for example the edge between floor and wall would also surpass
 * the red threshold together with the green and blue channels. Now, we are more specific searching for the red laser
 * line.
 */
void CImageManip::diff_red(CRawImage* laserImage, CRawImage* noLaserImage, CRawImage* diffImage) {

	int threshold = 20;
	int diff_threshold = 20;

	int w = laserImage->getwidth();
	assert (w > 0);
	int h = laserImage->getheight();
	assert (w == noLaserImage->getwidth());
	assert (h == noLaserImage->getheight());

	VALUE_TYPE* im_diff = diffImage->data;

	// x over width, y over height
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {

			bool d;
			int pos = 3*(y*w+x);
			d = isMoreRed(*laserImage, *noLaserImage, pos, threshold, diff_threshold);

			if (d) {
				im_diff[pos+0] = 200;
				im_diff[pos+1] = 200;
				im_diff[pos+2] = 200;
			} else {
				im_diff[pos+0] = 0;
				im_diff[pos+1] = 0;
				im_diff[pos+2] = 0;
			}
		}
	}
}

/**
 * Just use absolute difference for debugging.
 */
void CImageManip::diff_rgb(CRawImage* laserImage, CRawImage* noLaserImage, CRawImage* diffImage)
{
	int threshold = 20;
	int diff_threshold = 20;

	int w = laserImage->getwidth();
	assert (w > 0);
	int h = laserImage->getheight();
	assert (w == noLaserImage->getwidth());
	assert (h == noLaserImage->getheight());

	VALUE_TYPE* im_diff = diffImage->data;

	// x over width, y over height
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			int pos = 3*(y*w+x);
			im_diff[pos+0] = abs(laserImage->data[pos+0] - noLaserImage->data[pos+0]);
			im_diff[pos+1] = abs(laserImage->data[pos+1] - noLaserImage->data[pos+1]);
			im_diff[pos+2] = abs(laserImage->data[pos+2] - noLaserImage->data[pos+2]);
		}
	}
}

bool CImageManip::CheckIntegrity(CRawImage *img) {
	// For debugging loop over entire image
	bool result = true;
	int nonzeros = 0;
	for (int i = 0; i < img->getsize(); ++i) {
		if (img->data[i] != 0) nonzeros++;
	}
	if (nonzeros < 10) {
		fprintf(stderr, "There are only %i non-zero values in diff images, cannot be good\n", nonzeros);
		result = false;
	}
	return result;
}
