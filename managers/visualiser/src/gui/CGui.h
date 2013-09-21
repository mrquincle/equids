#ifndef __CGUI_H__
#define __CGUI_H__

#include "CRawImage.h"
#include <math.h>
#include <SDL/SDL.h>

class CGui
{
public:
  CGui();
  ~CGui();

  void drawImage(CRawImage* image);
  void drawStatus(bool *status);
  void initJockeys();
  void update();
  
private:
  SDL_Surface *screen;
  CRawImage *jockeyArray[20];
  CRawImage  activeArray[20];
  CRawImage  idleArray[20];
};

#endif

/* end of CGui.h */
