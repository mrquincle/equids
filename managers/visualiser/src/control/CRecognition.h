/*
 * File name: CRecognition.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CRECOGNITION_H__
#define __CRECOGNITION_H__

#include "CRawImage.h"
#include "CTimer.h"
#include <math.h>
#define MAX_SEGMENTS 10000
#define COLOR_PRECISION 32
#define COLOR_STEP 8

typedef struct{
	int x;
	int y;
	int size;
}SSegment;

typedef struct{
	int x;
	int y;
}SPixelPosition;

class CRecognition
{
public:
  CRecognition();
  ~CRecognition();
  SPixelPosition findMean(CRawImage* image);
  SPixelPosition findSegment(CRawImage* image);
  void learnPixel(unsigned char* a);
  void addPixel(unsigned char* a);

  void increaseTolerance();
  void decreaseTolerance();
  void resetColorMap();
  SPixelPosition findPath(CRawImage* image);

private:
  int tolerance;  
  float evaluatePixel1(unsigned char* a);
  float evaluatePixel2(unsigned char* a);
  float evaluatePixel3(unsigned char* a);
  int evaluatePixelFast(unsigned char *a);
  void rgbToHsv(unsigned char r, unsigned char  g, unsigned char b, unsigned int *h, unsigned char *s, unsigned char *v );

  unsigned char learned[3];
  unsigned int learnedHue;
  unsigned char learnedSaturation,learnedValue;
  unsigned char *colorArray;
  SSegment *segmentArray;
  bool debug;
  int *stack;
  int *buffer;
};

#endif

/* end of CRecognition.h */
