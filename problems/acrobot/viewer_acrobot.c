/* Copyright or © or Copr. 2012, Jean-François Hren
 *
 * Author e-mail: jean-francois.hren@inria.fr
 *
 * This software is a computer program whose purpose is to control
 * deterministic systems using optimistic planning.
 *
 * This software is governed by the CeCILL license under French law and
 * abiding by the rules of distribution of free software.  You can  use, 
 * modify and/ or redistribute the software under the terms of the CeCILL
 * license as circulated by CEA, CNRS and INRIA at the following URL
 * "http://www.cecill.info". 
 *
 * As a counterpart to the access to the source code and  rights to copy,
 * modify and redistribute granted by the license, users are provided only
 * with a limited warranty  and the software's author,  the holder of the
 * economic rights,  and the successive licensors  have only  limited
 * liability. 
 *
 * In this respect, the user's attention is drawn to the risks associated
 * with loading,  using,  modifying and/or developing or reproducing the
 * software by the user in light of its specific status of free software,
 * that may mean  that it is complicated to manipulate,  and  that  also
 * therefore means  that it is reserved for developers  and  experienced
 * professionals having in-depth computer knowledge. Users are therefore
 * encouraged to load and test the software's suitability as regards their
 * requirements in conditions enabling the security of their systems and/or 
 * data to be ensured and,  more generally, to use and operate it in the 
 * same conditions as regards security. 
 *
 * The fact that you are presently reading this means that you have had
 * knowledge of the CeCILL license and that you accept its terms.
 */

#define _XOPEN_SOURCE
#include <stdlib.h>
#undef _XOPEN_SOURCE

#include <stdio.h>
#include <math.h>

#include "acrobot.h"
#include "../viewer.h"

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>
#include <SDL/SDL_framerate.h>
#include <SDL/SDL_getenv.h>

static SDL_Surface *screen;				//Represent the screen

static FPSmanager fpsm;					//Control the fps

static int screenWidth;
static int screenHeight;

static void(*algorithm_drawingProcedure)(SDL_Surface*,int,int,void*);


/* Initialisation of the viewer. To be call before everything else. */

int initViewer(char* resolution, void(*drawingProcedure)(SDL_Surface*,int,int,void*), char isFullscreen) {

    const SDL_VideoInfo *info;
    Uint8  video_bpp;
    Uint32 videoflags;

    /* Initialize SDL */
    if(SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "Couldn't initialize SDL: %s\n",SDL_GetError());
        return -1;
    }

    atexit(SDL_Quit);

    putenv("SDL_VIDEO_WINDOW_POS=center");

    /* Alpha blending doesn't work well at 8-bit color */
    info = SDL_GetVideoInfo();

    if(info->vfmt->BitsPerPixel > 8)
        video_bpp = info->vfmt->BitsPerPixel;
    else
        video_bpp = 16;

    videoflags = SDL_HWSURFACE | SDL_SRCALPHA | SDL_RESIZABLE | SDL_DOUBLEBUF;

    if(isFullscreen)
        videoflags |= SDL_FULLSCREEN;

    /* Set 640x480 video mode */
    screenWidth = atoi(strtok(resolution, "x"));
    screenHeight = atoi(strtok(NULL, "x"));
    if((screen=SDL_SetVideoMode(screenWidth,screenHeight,video_bpp,videoflags)) == NULL) {
        fprintf(stderr, "Couldn't set %ix%i video mode: %s\n",screenWidth,screenHeight,SDL_GetError());
        return -1;
    }

    /* Use alpha blending */
    SDL_SetAlpha(screen, SDL_SRCALPHA, 0);

    /* Set title for window */
    SDL_WM_SetCaption("Ball Viewer","Ball Viewer");

    SDL_initFramerate(&fpsm);

    SDL_setFramerate(&fpsm, 1.0 / timeStep);

    algorithm_drawingProcedure = drawingProcedure;

    return 0;

}


/* Draw a ball with the defined color */

static void drawAcrobot(state* s, unsigned char r, unsigned char g, unsigned char b, unsigned char a) {

    double ratio = ((screenWidth / 2.0) - 10) / ((2.0 * parameters[0]) + (2.0 * parameters[3]));
    double x0 = (((screenWidth / 2.0) - 10) / 2.0) + 5;
    double y0 = ((screenHeight -10) / 2.0) + 5;
    double x1 = x0 + (sin(s->angularPosition1) * parameters[0] * ratio);
    double y1 = y0 - (cos(s->angularPosition1) * parameters[0] * ratio);
    double x2 = x1 + (sin(s->angularPosition2) * parameters[3] * ratio);
    double y2 = y1 - (cos(s->angularPosition2) * parameters[3] * ratio);

    aalineRGBA(screen, x0, y0, x1, y1, r, g, b, a);
    aalineRGBA(screen, x1, y1, x2, y2, r, g, b, a);
    aacircleRGBA(screen, x0, y0, 5, r, g, b, a);
    aacircleRGBA(screen, x1, y1, 5, r, g, b, a);

}


static int waitForAnotherSpace() {

    SDL_Event event;

    while(1) {
        SDL_framerateDelay(&fpsm);

        while(SDL_PollEvent(&event)) {
            if(event.type == SDL_KEYDOWN) {
                if(event.key.keysym.sym == SDLK_SPACE)
                    return 0;
                if(event.key.keysym.sym == SDLK_ESCAPE)
                    return 1;
            }
        }
    }

}


unsigned int viewer(state* s, double* a, double reward, void* instance) {

    char str[255];
    int done = 0;
    SDL_Event event;

    SDL_FillRect(screen, NULL, SDL_MapRGBA(screen->format, 255,255,255,255));

    if(a != NULL) {
        sprintf(str, "Applied Acceleration  : % f", ((parameters[7] + parameters[7]) * a[0]) - parameters[7]);
        stringRGBA(screen, 5, 5, str, 0, 0, 0, 255);
    }

    sprintf(str, "Angular Position 1 : % f", s->angularPosition1);
    stringRGBA(screen, 5, 15, str, 0, 0, 0, 255);

    sprintf(str, "Angular Position 2 : % f", s->angularPosition2);
    stringRGBA(screen, 5, 25, str, 0, 0, 0, 255);

    sprintf(str, "Reward          : % f", reward);
    stringRGBA(screen, 5, 35, str, 0, 0, 0, 255);

    drawAcrobot(s, 0, 0, 0, 255);

 
    if(a != NULL) {
        if(a[0] < 0.5)
            filledTrigonRGBA(screen, (screenWidth / 4.0) - 10, screenHeight * 0.9, (screenWidth / 4.0), (screenHeight * 0.9) - 5, (screenWidth / 4.0) - 10, (screenHeight * 0.9) - 10 , 0, 0, 0, 255);
        else if(a[0] > 0.5)
            filledTrigonRGBA(screen, (screenWidth / 4.0) + 10, screenHeight * 0.9, (screenWidth / 4.0) + 20, (screenHeight * 0.9) - 5, (screenWidth / 4.0) + 10, (screenHeight * 0.9) - 10, 0, 0, 0, 255);
        else
            boxRGBA(screen, (screenWidth / 4.0) - 5, screenHeight * 0.9, (screenWidth / 4.0) + 5, (screenHeight * 0.9) - 10, 0, 0, 0, 255);
    }

    if(algorithm_drawingProcedure != NULL)
        algorithm_drawingProcedure(screen, screenWidth, screenHeight, instance);

    SDL_Flip(screen);

    while(SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN:
                if(event.key.keysym.sym == SDLK_ESCAPE)
                    done = 1;
                if(event.key.keysym.sym == SDLK_SPACE)
                    done = waitForAnotherSpace();
                break;
	
            case SDL_QUIT:
                done = 1;
            break;

        }
    }

    SDL_framerateDelay(&fpsm);

    return done;

}
