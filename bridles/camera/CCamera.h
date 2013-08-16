/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Robot camera
 * @file CCamera.h
 *
 * This file is created at Czech Technical Unversity, Prague.
 *
 * Copyright (c) 2013 Tom Krajnik
 *
 * @author      Tom Krajnik
 * @date        Jan 18, 2011
 * @project     Replicator
 * @university  Czech Technical Unversity, Prague.
 * @case        Navigation
 */

#ifndef __CCAMERA_H__
#define __CCAMERA_H__
#include "CRawImage.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>


class CCamera {
public:
	bool initialized;

	CCamera();

	//! If you want to initialize a real camera
	int Init(const char *deviceName, int &devfd, int width, int height);

	//! If you want to load images from a directory use this "dummy" camera
	int dummyInit(const char *directoryName, const char *prefixImage);

	void Stop();

	~CCamera();

	//! This gets you a new image, by default it will convert it to RGB values
	int renewImage(CRawImage* image, bool convert);

	//! Denoise image by capturing another image and averaging over the two
	int denoiseImageByCapturingAnother(CRawImage* image);

	void setGain(int value);
	int getGain();

	void setExposition(int value);
	int getExposition();

	void setBrightness(int value);
	int getBrightness();
	int setDeviceAutoExposure(const bool val);

	void yuv422_to_rgb(unsigned char *output_ptr, unsigned char *input_ptr, size_t width_times_height);

	void yuyv_to_uyvy(unsigned char *data, size_t width, size_t height);

	inline void saveImages(bool save) { save_images = save; }
protected:
	//! This gets you a dummy image
	int dummyImage(CRawImage* image);

private:
	char dummy_mode;
	CRawImage defaultImage;
	int exposition;
	int brightness;
	int gain;
	char directory[2000];
	char loadFilePrefix[16];
	int loadFileIndex;
	int saveFileIndex;
	struct vdIn *videoIn;
	int width, height;
	bool save_images;
	int camdevfd;

	int pixel_format;
	bool print_debug;
};

#endif
