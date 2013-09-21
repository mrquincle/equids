#include "CGui.h"

#define THICK_CROSS

CGui::CGui()
{
  SDL_Init(SDL_INIT_VIDEO);
  screen = NULL;
  screen = SDL_SetVideoMode(1240,480,24,SDL_SWSURFACE); 
  if (screen == NULL)fprintf(stderr,"Couldn't set SDL video mode: %s\r\n",SDL_GetError());
  SDL_WM_SetCaption("Robot revue vision system","Robot revue vision system");
  initJockeys();
}

CGui::~CGui()
{
}

void CGui::initJockeys()
{
	char dummy[20];
	for (int i = 0;i<20;i++){
		activeArray[i].width=300;
		activeArray[i].height=48;
		sprintf(dummy,"images/A%04i.bmp",i);
		activeArray[i].loadBmp(dummy);
		sprintf(dummy,"images/I%04i.bmp",i);
		idleArray[i].width=300;
		idleArray[i].height=48;
		idleArray[i].loadBmp(dummy);
	}
}


void CGui::drawImage(CRawImage* image)
{
	int result = 0;
	SDL_Rect rect;				
	rect.x = 300;
	rect.y = 0;
	rect.w = 640;
	rect.h = 480;
	SDL_Surface *imageSDL = SDL_CreateRGBSurfaceFrom(image->data,image->width,image->height,image->bpp*8,image->bpp*image->width,0x000000ff,0x0000ff00,0x00ff0000,0x00000000);
	if (imageSDL != NULL && SDL_BlitSurface(imageSDL, NULL, screen, &rect)==0) result = 0;
	SDL_FreeSurface(imageSDL);
}

void CGui::drawStatus(bool *status)
{
	int result = 0;
	SDL_Rect rect;				
	rect.x = 0;
	rect.y = 0;
	rect.w = 300;
	rect.h = 48;
	for (int i =0;i<20;i++){
		if (status[i]) jockeyArray[i] = &activeArray[i]; else jockeyArray[i] = &idleArray[i];
	}
	SDL_FillRect(screen, &rect, SDL_MapRGB(screen->format, 0, 0, 0));
	CRawImage *image;
	for (int i =0;i<20;i++){
		rect.x = (i/10)*940;
		rect.y = (i%10)*48;
		CRawImage* image=jockeyArray[i];
		SDL_Surface *imageSDL = SDL_CreateRGBSurfaceFrom(image->data,image->width,image->height,image->bpp*8,image->bpp*image->width,0x000000ff,0x0000ff00,0x00ff0000,0x00000000);
		if (imageSDL != NULL && SDL_BlitSurface(imageSDL, NULL, screen, &rect)==0) result = 0;
		SDL_FreeSurface(imageSDL);
	}
}

void CGui::update()
{
  SDL_UpdateRect(screen,0,0,0,0);
}
