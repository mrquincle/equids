/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CImageManip.h
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

#ifndef CIMAGEMANIP_H_
#define CIMAGEMANIP_H_

#include <CRawImage.h>

enum ColorSpace { CS_RGB, CS_BGR};

enum Color { C_RED, C_GREEN, C_BLUE};

/**
 * Some general helper functions around CRawImage.
 */
class CImageManip {
public:
	CImageManip(ColorSpace colorSpace) {
		setColorSpace(colorSpace);
	}

	~CImageManip() {};

	void setColorSpace(ColorSpace colorSpace);

	//!
	bool greater_than(CRawImage &img1, CRawImage &img2, int pos, Color color);

	//! Compare pixels
	bool isMoreRed(CRawImage &img1, CRawImage &img2, int pos, int threshold, int diff_threshold);

	//! Return difference between two images
	void diff_red(CRawImage* laserImage, CRawImage* noLaserImage, CRawImage* diffImage);
	void diff_rgb(CRawImage* laserImage, CRawImage* noLaserImage, CRawImage* diffImage);

	/**
	 * Check the integrity of an image. Can be detection of the different types of noise. In this case for difference
	 * images it is important that the difference image contains actual data, so it should have a sufficient number of
	 * non-zero values.
	 */
	bool CheckIntegrity(CRawImage *img);

private:
	ColorSpace colorSpace;

	Color channel_color[3];

	char channel_index[3];

};


#endif /* CIMAGEMANIP_H_ */
