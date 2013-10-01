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
#include <algorithm>
#include <iostream>
#include <sys/syslog.h>

//! Declare the following as unmangled C functions
extern "C" {
#include <grab.h>
#include <libcam2.h>
}

#include <CCamera.h>

#define ASSERT(condition) { \
		if(!(condition)){ \
			std::cerr << "ASSERT FAILED: " << #condition << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; \
			assert(condition); \
		} \
}

#define ASSERT_EQUAL(x,y) \
		if (x != y) { \
			std::cout << #x << " != " << #y ", specifically: " << x << " != " << y << std::endl; \
			assert(x == y); \
		}


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

#define DEFAULT_IMAGE_WIDTH  640
#define DEFAULT_IMAGE_HEIGHT 480

//! This is a crappy implementation from Sheffield that does grab one image at a time
#define OLD
// kernel panic, needs debugging: #undef OLD

/**
 * Create a default camera device.
 */
CCamera::CCamera(): width(DEFAULT_IMAGE_WIDTH), height(DEFAULT_IMAGE_HEIGHT),
		defaultImage(CRawImage(DEFAULT_IMAGE_WIDTH,DEFAULT_IMAGE_HEIGHT,3))
{
	log_prefix = "CCamera: "; // call setLogPrefix with a string with you controller embedded
	gain = exposition = 0;
	loadFileIndex = 0;
	saveFileIndex = 0;
	dummy_mode = 0;
	videoIn = 0;
	brightness = 0;
	initialized = false;
	save_images = false;
	camdevfd = -1;
	pixel_format = V4L2_PIX_FMT_YUYV;
	log_level = LOG_EMERG;
	semaphore_set = false;
	flip_camera = false;
	ASSERT_EQUAL(defaultImage.getwidth(), width);
    stopped = true;
}

CCamera::~CCamera() {
	Stop();
}

/**
 * Set camera parameters.
 *
 * @param deviceName         v4l or v4l2 camera device, e.g. /dev/video0
 * @param devfd              file descriptor of camera device
 * @param width              resolution
 * @param height             resolution
 * @return                   success (0), failure (<0)
 */
int CCamera::Init(int width, int height)
{
	this->width = width;
	this->height = height;
#ifndef OLD
	init_mmap(devfd, deviceName);
	start_capturing(devfd);
#endif

	// use an environmental variable FLIP_CAMERA, by default flip camera is false
	flip_camera = false;
	char *str_flip_camera = getenv("FLIP_CAMERA");
	if (str_flip_camera) {
		std::string s = std::string(str_flip_camera);
		std::transform(s.begin(), s.end(), s.begin(), ::tolower);
		if (s == "true") {
			flip_camera = true;
		}
	}

	if (flip_camera) {
		printf("%sCamera will be flipped\n", log_prefix.c_str());
	} else {
		printf("%sCamera will not be flipped\n", log_prefix.c_str());
	}

	// we will need to capture a few images to get rid of the greenish pictures in the beginning
	// we cannot do that anymore, because we are not allowed to open the device on Init
//	printf("%s We capture 10 images to get rid of greenish pictures in the beginning\n", log_prefix.c_str());
//	for (int i = 0; i < 10; ++i) {
//		cam_capture(camdevfd, width, height);
//	}
	return 0;
}

void CCamera::setSemaphore(sem_t *cap_sem) {
	capture_sem = cap_sem;
	semaphore_set = true;
}

/**
 * Initialize the folder to get the dummy images from. It should have a bunch of images starting with "0000.bmp" and
 * incrementing from there, "0001.bmp", etc.
 *
 * @param directoryName      directory from where to capture *.bmp files, expects first file "0000.bmp"
 * @return                   success (0), failure (-1)
 */
int CCamera::dummyInit(const char *directoryName, const char *prefixImage) {
	dummy_mode = 1;
	char fileName[1000];
	strcpy(directory, directoryName);
	strcpy(loadFilePrefix, prefixImage);
	sprintf(fileName, "%s/%s0000.bmp", directory, loadFilePrefix, prefixImage);
	fprintf(stderr,"%sCamera type: dummy camera\n", log_prefix.c_str());
	FILE* file = fopen(fileName,"r");
	if (file == NULL){
		fprintf(stderr,"File %s not found.\n",fileName);
		return -1;
	}
	return 0;
}

/**
 * Closes the file descriptor to the camera.
 */
void CCamera::Stop() {
	if (stopped) {
		printf("%sCamera is already stopped\n", log_prefix.c_str());
		return;
	} else {
		printf("%sStop camera \n", log_prefix.c_str());
	}

	if (camdevfd >= 0) {
		cam_closedev(camdevfd);
	} else {
		printf("%sCould not stop camera, no proper device handler \n", log_prefix.c_str());
	}
	camdevfd = -1;
//	if (log_level >= LOG_INFO)
//		printf("Camera device handler closed\n");
	stopped = true;
}

/**
 * Start the camera actually, opens the device
 *
 * @param deviceName         v4l or v4l2 camera device, e.g. /dev/video0
 * @param devfd              file descriptor of camera device
 */
int CCamera::Start(const char *deviceName, int &devfd) {
	if (!stopped) {
		printf("%sCamera is already started\n", log_prefix.c_str());
		return 0;
	} else {
		printf("%sStart camera \n", log_prefix.c_str());
	}

	if (log_level >= LOG_INFO)
		printf("%sOpen device %s\n", log_prefix.c_str(), deviceName);
	stopped = false;
	camdevfd = cam_opendev(deviceName, width, height, 1);
	this->width = width;
	this->height = height;
	if (!camdevfd) {
		fprintf(stderr, "%sCannot (re)open video device.\n", log_prefix.c_str());
		return -1;
	} else {
		if (log_level >= LOG_INFO)
			printf("%sDevice %s opened\n", log_prefix.c_str(), deviceName);
	}
	devfd = camdevfd;
	return 0;
}

/**
 * Fill a previously allocated image with new data from the buffer in the v4l camera driver. The data in the buffer is
 * of a YUYV pattern. Originally, the sensor image has - of course - a Bayer pattern, for more information see the nice
 * wikipedia article: https://en.wikipedia.org/wiki/Bayer_filter. This pattern has twice as many green pixels as red and
 * blue ones, because of the sensitivity of the human eye for green.
 *
 * Anyway, the YUYV pattern has 4 values per in the end 2 RGB pixels. So, by asking the v4l or v4l2 drivers to return
 * images in the YUYV (or YYUV, etc.) format, we get an image buffer of the size width*height*2. Now, we use these
 * values twice to create in the end two RGB pixels. This is done in the yuv422_to_rgb function.
 *
 * @param image              the image to be returned
 * @param convert            convert Bayer pattern to RGB, recommended!
 * @return                   success (0), failure (<0)
 */
int CCamera::renewImage(CRawImage* image, bool convert, bool swap)
{
	if (semaphore_set) {
		std::cout << "Wait for semaphore (from e.g. streaming thread)" << std::endl;
		sem_wait(capture_sem);
	}

	if (dummy_mode) return dummyImage(image);

	size_t yuv_size = width*height*2;
	if (log_level >= LOG_INFO)
		printf("Size of YUVY is %i\n", (int)yuv_size);
	assert (yuv_size > 0);
	unsigned char* buffer = NULL;

#ifdef OLD
	buffer = cam_capture(camdevfd, width, height);
#else
	buffer = cam_stream(camdevfd);
#endif

	if (log_level >= LOG_INFO)
		printf("%sGrabbed frame, now copy to buffer in CRawImage\n", log_prefix.c_str());

	if (save_images) {
		char fileName[256];
		sprintf(fileName, "%s%04i.jpg", "image", ++saveFileIndex);
		//		yuyv_to_uyvy((unsigned char*)b.start, width, height); // jpeg function does not read YUYV format properly
		fprintf(stderr, "%sRemoved jpeg support. Seems overkill on a robot.\n", log_prefix.c_str());
		//		save_jpeg_image((unsigned char*)b.start, height, width, fileName);
		//		yuyv_to_uyvy((unsigned char*)b.start, width, height); // transform back
	}

	if (convert) {
		yuv422_to_rgb(image->data, (unsigned char*)buffer, yuv_size);
	} else {
		fprintf(stderr, "%sJust realize that you copied the original YUV formatted data.\n", log_prefix.c_str());
		memcpy(image->data,buffer,yuv_size);
	}

	if(swap){
		image->swap(CC_RED, CC_BLUE);
	}

	return 0; 
}

/**
 * Denoise image by capturing another one and averaging over the two. This is of course a very blunt way with coping
 * with noisy images. The most typical noise is a sudden "green" horizontal line. Getting rid of the images that have
 * such lines will be most useful.
 *
 * @param image              image to be returned
 * @return                   success (0), or failure (<0)
 */
int CCamera::denoiseImageByCapturingAnother(CRawImage* image)
{
	if (dummy_mode) return dummyImage(image);

	size_t yuv_size = width*height*2;
	if (log_level >= LOG_INFO)
		printf("Size is %i\n", (int)yuv_size);
	assert (yuv_size > 0);
	unsigned char* buffer = NULL;
	buffer = cam_capture(camdevfd, width, height);

	if (log_level >= LOG_INFO)
		printf("%sGrabbed frame, now copy to buffer in CRawImage\n", log_prefix.c_str());

	yuv422_to_rgb(defaultImage.data, (unsigned char*)buffer, yuv_size);

	image->average(defaultImage);

	return 0;
}

/**
 * Not always is it useful to use the camera, for example when it is broken, or when you compile this code on the host.
 * In that case, you can use this function to return a dummy image from a directory with images which simulates the
 * proper stuff.
 *
 * @param image              next dummy image from the directory
 * @return                   success (0), failure (<0)
 */
int CCamera::dummyImage(CRawImage* image)
{
	char fileName[1000];
	sprintf(fileName,"%s/%s%04i.bmp",directory,loadFilePrefix,loadFileIndex);
	printf("%sTries to load file %s as a dummy image\n", log_prefix.c_str(), fileName);
	if (image->loadBmp(fileName)) {
		loadFileIndex++;
	} else {
		loadFileIndex=0;
		sprintf(fileName,"%s/%s%04i.bmp",directory,loadFilePrefix,loadFileIndex);
		image->loadBmp(fileName);
	}

	ASSERT_EQUAL(image->getwidth(),width);
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
 * There is no gamma correction in this function, and no other sophisticated elaborations either. Take in mind that this
 * is mainly meant for a robot, not for a human.
 *
 * @param output_ptr         start of the output buffer
 * @param input_ptr          start of the input buffer
 */
void CCamera::yuv422_to_rgb(unsigned char * output_ptr, unsigned char * input_ptr, size_t width_times_height)
{
	if (log_level >= LOG_INFO)
		printf("%sConvert yuv to rgb\n", log_prefix.c_str());

	unsigned int i, size;
	unsigned char Y0, Y1, U, V;
	unsigned char *buff = input_ptr;
	unsigned char *output_pt = output_ptr;

	size = width_times_height / 4;  // make the size 640x480 / 2 and fill output with buffer of size 6x

	if (flip_camera) {
		output_pt += 6*size - 1;
	}

	for (i = 0; i < size; ++i) {
		// pick the Y, Y, U, V pixels from the right places
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

		if (flip_camera) {
			*output_pt-- = YUV2B(Y0, U, V);
			*output_pt-- = YUV2G(Y0, U, V);
			*output_pt-- = YUV2R(Y0, U, V);

			*output_pt-- = YUV2B(Y1, U, V);
			*output_pt-- = YUV2G(Y1, U, V);
			*output_pt-- = YUV2R(Y1, U, V);

		} else {
			// make two RGB pixels from the YYUV values

			*output_pt++ = YUV2R(Y0, U, V); // red is at the right place, does have nothing to do with "U", only with Y0 and V
			*output_pt++ = YUV2G(Y0, U, V); // too green
			*output_pt++ = YUV2B(Y0, U, V); // too yellow

			*output_pt++ = YUV2R(Y1, U, V);
			*output_pt++ = YUV2G(Y1, U, V);
			*output_pt++ = YUV2B(Y1, U, V);
		}
	}

	if (flip_camera) {
		assert((output_pt - output_ptr) == -1);
	} else {
		if (log_level >= LOG_INFO)
			printf("%sCompare %i with %i\n", log_prefix.c_str(), (int)(output_pt - output_ptr), (int)(height*width*3));
		assert((output_pt - output_ptr) == height*width*3);
	}
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
