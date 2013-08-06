/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Camera functionality on a robot
 * @file CCamera.cpp
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

#include <cstdio>
#include <cstdlib>
#include <cassert>

//! Declare the following as unmangled C functions
extern "C" {
#include <grab.h>
#include <libcam2.h>
}

#include <CCamera.h>


//-----------------------------------------------------------------------------
// COLOR PART
//-----------------------------------------------------------------------------

#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)

// RGB -> YUV
#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)

// YUV -> RGB
#define C(Y) ( (Y) - 16  )
#define D(U) ( (U) - 128 )
#define E(V) ( (V) - 128 )

#define YUV2R(Y, U, V) CLIP(( 298 * C(Y)              + 409 * E(V) + 128) >> 8)
#define YUV2G(Y, U, V) CLIP(( 298 * C(Y) - 100 * D(U) - 208 * E(V) + 128) >> 8)
#define YUV2B(Y, U, V) CLIP(( 298 * C(Y) + 516 * D(U)              + 128) >> 8)

// RGB -> YCbCr
#define CRGB2Y(R, G, B) CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16)
#define CRGB2Cb(R, G, B) CLIP((36962 * (B - CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16) ) >> 16) + 128)
#define CRGB2Cr(R, G, B) CLIP((46727 * (R - CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16) ) >> 16) + 128)

// YCbCr -> RGB
#define CYCbCr2R(Y, Cb, Cr) CLIP( Y + ( 91881 * Cr >> 16 ) - 179 )
#define CYCbCr2G(Y, Cb, Cr) CLIP( Y - (( 22544 * Cb + 46793 * Cr ) >> 16) + 135)
#define CYCbCr2B(Y, Cb, Cr) CLIP( Y + (116129 * Cb >> 16 ) - 226 )


//-----------------------------------------------------------------------------
CCamera::CCamera(): defaultImage(CRawImage(640,480,3))
{
	gain = exposition = 0;
	width = height = 0;
	loadFileIndex = 0;
	saveFileIndex = 0;
	dummy_mode = 0;
	videoIn = 0;
	brightness = 0;
	initialized = false;
	save_images = false;
	camdevfd = -1;
	pixel_format = V4L2_PIX_FMT_YUYV;
	print_debug = false;
	return;
}

CCamera::~CCamera() {
	Stop();
}

int CCamera::Init(const char *deviceName, int &devfd, int width, int height)
{
	if (print_debug)
		printf("CCamera: Open device %s\n", deviceName);
	//devfd = open_device();
	camdevfd = cam_opendev(deviceName, width, height, 1);
	this->width = width;
	this->height = height;
	if (!camdevfd) {
		printf("CCamera: Cannot open video device\n");
		return -1;
	} else {
		if (print_debug)
			printf("CCamera: Device %s opened\n", deviceName);
	}
	devfd = camdevfd;

	// we will need to capture a few images to get rid of the greenish pictures in the beginning
	for (int i = 0; i < 10; ++i) {
		cam_capture(camdevfd, width, height);
	}
	return 0;
}

int CCamera::dummyInit(const char *directoryName) {
	dummy_mode = 1;
	char fileName[1000];
	strcpy(directory,directoryName);
	sprintf(fileName,"%s/0000.bmp",directory);
	fprintf(stderr,"Camera type: dummy camera\n");
	FILE* file = fopen(fileName,"r");
	if (file == NULL){
		fprintf(stderr,"File %s not found.\n",fileName);
		return -1;
	}
	return 0;
}

void CCamera::Stop() {
	if (camdevfd >= 0) cam_closedev(camdevfd);
	camdevfd = -1;
	if (print_debug)
		printf("Camera device handler closed\n");
}

/**
 * Fill a previously allocated image with new data from the buffer in the v4l camera driver. This
 */
int CCamera::renewImage(CRawImage* image, bool convert)
{
	if (dummy_mode) return dummyImage(image);

	size_t yuv_size = width*height*2;
	if (print_debug)
		printf("Size is %i\n", (int)yuv_size);
	assert (yuv_size > 0);
	unsigned char* buffer = NULL;
	buffer = cam_capture(camdevfd, width, height);

	if (print_debug)
		printf("Grabbed frame, now copy to buffer in CRawImage\n");

	if (save_images) {
		char fileName[256];
		sprintf(fileName, "%s%04i.jpg", "image", ++saveFileIndex);
//		yuyv_to_uyvy((unsigned char*)b.start, width, height); // jpeg function does not read YUYV format properly
		fprintf(stderr, "Removed jpeg support. Seems overkill on a robot.\n");
//		save_jpeg_image((unsigned char*)b.start, height, width, fileName);
//		yuyv_to_uyvy((unsigned char*)b.start, width, height); // transform back
	}

	if (convert) {
		yuv422_to_rgb(image->data, (unsigned char*)buffer, yuv_size);
	} else {
		fprintf(stderr, "Just realize that you copied the original Bayer pattern formatted data.\n");
		memcpy(image->data,buffer,yuv_size);
	}
	return 0; 
}

/**
 * Denoise image by capturing another one and averaging over the two.
 */
int CCamera::denoiseImageByCapturingAnother(CRawImage* image)
{
	if (dummy_mode) return dummyImage(image);

	size_t yuv_size = width*height*2;
	if (print_debug)
		printf("Size is %i\n", (int)yuv_size);
	assert (yuv_size > 0);
	unsigned char* buffer = NULL;
	buffer = cam_capture(camdevfd, width, height);

	if (print_debug)
		printf("Grabbed frame, now copy to buffer in CRawImage\n");

	yuv422_to_rgb(defaultImage.data, (unsigned char*)buffer, yuv_size);

	image->average(defaultImage);

	return 0;
}


int CCamera::dummyImage(CRawImage* image)
{
	char fileName[1000];
	sprintf(fileName,"%s/%04i.bmp",directory,loadFileIndex);
	printf("Tries to load file %s\n", fileName);
	if (image->loadBmp(fileName)) {
		loadFileIndex++;
	} else {
		loadFileIndex=0;
		sprintf(fileName,"%s/%04i.bmp",directory,loadFileIndex);
		image->loadBmp(fileName);
	}
	assert (image->getwidth() == width);
	return 0;
}


/**
 * According to: v4l2-ctl --list-formats on the robot itself:
    ioctl: VIDIOC_ENUM_FMT
        Index       : 0
        Type        : Video Capture
        Pixel Format: 'YUYV'
        Name        : YCbCr 4:2:2 Interleaved YUYV
 * This means there should be the pixel format "V4L2_PIX_FMT_YUYV" used in the grab.cpp file. Hence, here we also use
 * it in that order: Y0 U Y1 V.
 *
 * There is no gamma correction in this function
 */
void CCamera::yuv422_to_rgb(unsigned char * output_ptr, unsigned char * input_ptr,
		size_t width_times_height)
{
	if (print_debug)
		printf("Convert yuv to rgb\n");
	unsigned int i, size;
	unsigned char Y0, Y1, U, V;
	unsigned char *buff = input_ptr;
	unsigned char *output_pt = output_ptr;
	size = width_times_height / 4;  // make the size 640x480 / 2 and fill output with buffer of size 6x
	for (i = 0; i < size; ++i) {
		if (pixel_format == V4L2_PIX_FMT_YUYV) {
			Y0 = buff[0];
			U  = buff[1];
			Y1 = buff[2];
			V  = buff[3];
		} else if (pixel_format == V4L2_PIX_FMT_UYVY) {
			U  = buff[0];
			Y0 = buff[1];
			V  = buff[2];
			Y1 = buff[3];
		} else if (pixel_format == V4L2_PIX_FMT_YYUV) {
			Y0 = buff[0];
			Y1 = buff[1];
			U  = buff[2];
			V  = buff[3];
		}
		buff += 4;

		*output_pt++ = YUV2R(Y0, U, V); // red is at the right place, does have nothing to do with "U", only with Y0 and V
		*output_pt++ = YUV2G(Y0, U, V); // too green
		*output_pt++ = YUV2B(Y0, U, V); // too yellow

		*output_pt++ = YUV2R(Y1, U, V);
		*output_pt++ = YUV2G(Y1, U, V);
		*output_pt++ = YUV2B(Y1, U, V);
	}
	if (print_debug)
		printf("Compare %i with %i\n", (int)(output_pt - output_ptr), (int)(height*width*3));
	assert((output_pt - output_ptr) == height*width*3);
}

void swap(unsigned char *x, unsigned char *y) {
	unsigned char t;
	t = *x;
	*x = *y;
	*y = t;
}

/**
 * Transform my default laptop webcam setting of YUYV (check with v4l2-ctl --list-formats) to UYVY which is required by the
 * jpeg.c functions, see void read_422_format() in that file. The Replicator bots are also in YUYV format by the way.
 */
void CCamera::yuyv_to_uyvy(unsigned char *data, size_t width, size_t height)
{
	unsigned char *ptr;
	unsigned long count;
	unsigned long i;

	ptr = (unsigned char *)(data);

	count = (unsigned long)((height*width) / 2);
	assert (count > 0);

	// we inspect every XXXX elements (4 chars)
	for (i=0; i<count; i++) {
		swap(ptr+0, ptr+1);
		swap(ptr+2, ptr+3);
		ptr+=4; // go to next byte
	}
}


int CCamera::getGain()
{
	fprintf(stderr,"Gain setting not supported by dummy camera.\n");
	return 0;
}

int CCamera::getExposition()
{
	fprintf(stderr,"Exposure setting not supported by dummy camera.\n");
	return 0;
}

int CCamera::getBrightness()
{
	fprintf(stderr,"Brightness setting not supported by dummy camera.\n");
	return 0;
}

void CCamera::setGain(int value)
{
	fprintf(stderr,"Gain setting not supported by dummy camera.\n");
}

void CCamera::setExposition(int exp)
{
	fprintf(stderr,"Exposure  setting not supported by dummy camera.\n");
}

void CCamera::setBrightness(int val)
{
	fprintf(stderr,"Brightness setting not supported by dummy camera.\n");
}
