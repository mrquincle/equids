/**
 * @brief Basic image storage and a few functions for manipulation
 * @file CRawImage.h
 *
 * Copyright © 2012 Tom Krajnik
 *
 * @author		Tom Krajnik, Anne C. van Rossum
 * @date		May 14th, 2013
 * @institute	Czech Technical University in Prague
 * @company		Almende B.V.
 * @project		Replicator
 */

#ifndef CIMAGE_H
#define CIMAGE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**
 * We do not want to use C++ templates, but on the other hand we want to change type at once.
 */
typedef unsigned char VALUE_TYPE;

enum ColorChannel { CC_RED = 0, CC_GREEN = 1, CC_BLUE = 2};

enum Orientation { O_VERTICAL, O_HORIZONTAL };

/**
 * This is a very basic implementation of an image. It just uses three char's, one for each of the color channels.
 * Nothing fancy, no padding. Moreover, this struct is only used for communication with the user. The internal structure
 * is a (linear) array with RGB values or monochrome values.
 */
struct Pixel {
	VALUE_TYPE r;
	VALUE_TYPE g;
	VALUE_TYPE b;
	Pixel(): r(0), g(0), b(0) {};
	Pixel(VALUE_TYPE r, VALUE_TYPE g, VALUE_TYPE b): r(r), g(g), b(b) {};

	Pixel operator=(const Pixel & other) {
		return Pixel((other.r),(other.g),(other.b));
	}

	Pixel operator+(const Pixel & other) {
		return Pixel((r+other.r),(g+other.g),(b+other.b));
	}

	Pixel operator/(char factor) {
		return Pixel(r/factor,g/factor,b/factor);
	}
};

/**
 * The patch is another convenient struct for the user. It allows you to get a patch with data from the image. It is
 * not meant to be super fast, it is just simple. It requires you to set a width and height beforehand, and then it
 * copies the pixels to the data field. This data field does not need to be allocated by you, this will be done in this
 * compilation unit.
 */
struct Patch {
	VALUE_TYPE *data;
	int width;
	int height;
	Patch(): data(NULL), width(0), height(0) {};
	Patch(int width, int height) {
		data = NULL;
		init(width, height);
	}
	void init(int width, int height, int bpp=3) {
		printf("%s(): initialize patch of size %i\n", __func__, width*height*bpp);
		this->width = width;
		this->height = height;
		printf("%s(): what is weird about new or calloc here?\n", __func__);
		data = new VALUE_TYPE[width*height*bpp];
		printf("%s(): initialized patch of size %i\n", __func__, width*height*bpp);
	}
	void free() {
		if (data != NULL) delete [] data;
	}
	~Patch() {
		free();
	}
};

/**
 * @author Tom Krajnik
 * @author Anne C. van Rossum
 */
class CRawImage {
public:

	//! The default constructor requires width, height, and number of bytes per pixel. The latter is only allowed to be
	//! one or three.
	CRawImage(int wi, int he, int bpp);

	//! Default destructor, will free internal "data" if not null.
	~CRawImage();

	//! Copy constructor
	CRawImage(const CRawImage & other);

	//! Average two images, write result to this image
	void average(const CRawImage & other);

	//! Gets value at [x,y], only valid if bpp=1
	VALUE_TYPE getValue(int x, int y);

	//! Sets value at [x,y], only valid if bpp=1
	void setValue(int x, int y, VALUE_TYPE value);

	//! Set a value in a given patch at [x,y], only valid if bpp=1
	void setValue(int x, int y, Patch & patch, VALUE_TYPE value);

	//! Only valid if bpp=3, returns an rgb pixel
	Pixel getPixel(int x, int y);

	//! Returns a pixel in a given patch
	Pixel getPixel(int x, int y, Patch & patch);

	//! Set a pixel in a given patch
	void setPixel(int x, int y, Patch & patch, Pixel pixel);

	//! Set a pixel in the picture
	void setPixel(int x, int y, Pixel pixel);

	//! Swap pixel(!) at [x0,y0] with [x1,y1], only valid if bpp=3!
	void swap(int x0, int y0, int x1, int y1);

	//! Swap pixel p0 with p1
	void swap(Pixel p0, Pixel p1);

	//! Set the patch
	void setPatch(int p_x, int p_y, Patch &patch);

	//! Get the patch itself (requires a malloc op)
	void getPatch(int x, int y, Patch & patch);

	//! Add header information etc. required to make a full-fledged CRawImage from a Patch struct
	CRawImage* patch2Img(Patch &patch);

	//! Get a patch the size of the picture divided by some factor
	void compress(Patch &patch);

	//! Clear all pixels (set them to zero), does not allocate or deallocate anything
	void clear();

	//! Reallocate internal data structures if necessary
	void refresh();

	//! Save to file, filename will automatically be appended by index
	void saveBmp(const char* name);

	//! With instances of class CRawImage across multiple binaries, the images will overwrite each other
	void saveNumberedBmp(const char* name, bool increment = true);

	//! Load the saved images
	bool loadNumberedBmp(const char* name, bool increment = true);

	//! Get the last file name
	int getSaveNumber();

	//! Get the last number/index of saved file
	static int numSaved;

	//! Get the last number/index of loaded file
	static int numLoaded;

	//! Load the image from file again, no index will be appended to the name
	bool loadBmp(const char* name);

	//! Swap the R and B channel if the image contains RGB values
	void swap(const ColorChannel channel1, const ColorChannel channel2);

	//! Flip vertically or horizontally
	void flip(const Orientation orientation);

	//! Set the right BMP header, depends on dimensions, bpp, etc.
	void updateHeader();

	//! Plot a vertical line through x (not entirely trivial if bpp > 1)
	void plotVerticalLine(int x);
	//! Plot a horizontal line through y (not entirely trivial if bpp > 1)
	void plotHorizontalLine(int y);

	//! Plot line from [x0,y0] to [x1,y1], only working for bpp=1 for now
	void plotLine(int x0, int y0, int x1, int y1);
	//! Plot blob in center of image
	void plotCenter();
	//! Plot a cross of size "size" at given position
	void plotCross(int i,int j,int size);

	//! Check if image is (already) monochrome
	inline bool isMonochrome() { return (bpp == 1); }

	//! This really throws information away! The grayscale values are according to human sensitivity values (30, 59, 11)
	void makeMonochrome();

	//! Set the brush color
	inline void setBrush(Pixel color) { brush = color; }

	/**
	 * Same as makeMonochrome, but keeps image intact, and stores result in parameter "result"
	 * @param result   Previously constructed image
	 */
	void makeMonochrome(CRawImage* result);

	//! Get the brightness by iterating over all pixels (expensive)
	double getOverallBrightness(bool upperHalf);

	//! Just show the data to the user, if you screw up, it's your own responsibility
	VALUE_TYPE* data;

	//! Reallocate the internal data structure on changing the number of bytes per pixel
	void setbpp(int bpp) { this->bpp = bpp; refresh(); }

	//! Reallocate the internal data structure on changing the image dimensions
	void setdimensions(int width, int height) { this->width = width; this->height = height; refresh(); }

	//! Get the width in pixels
	const inline int getwidth() { return width; };

	//! Get the height in pixels
	const inline int getheight() { return height; };

	//! Get the size in memory (pixels times the bytes per pixel)
	const inline int getsize() { return size; };
private:
	//! Width of image in pixels
	int width;
	//! Height of image in pixels
	int height;
	//! Bytes per pixel
	int bpp;
	//! Size of data structure (pixels times the bytes per pixel)
	int size;

	//! Brush with this color
	Pixel brush;

	//! To swap the color channels, or not
	bool do_swap;
};

#endif
