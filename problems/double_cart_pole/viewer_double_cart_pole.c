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
#define __USE_GNU
#include <math.h>
#undef __USE_GNU

#include "double_cart_pole.h"
#include "../viewer.h"

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>
#include <SDL/SDL_framerate.h>
#include <SDL/SDL_getenv.h>

#define NB_SPRING 5

static SDL_Surface *screen;

static FPSmanager fpsm;

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
        fprintf(stderr, "Couldn't set %ix%i video mode: %s\n",screenWidth, screenHeight,SDL_GetError());
        return -1;
    }

    /* Use alpha blending */
    SDL_SetAlpha(screen, SDL_SRCALPHA, 0);

    /* Set title for window */
    SDL_WM_SetCaption("Double Cart Pole Viewer","Double Cart Pole Viewer");

    SDL_initFramerate(&fpsm);

    SDL_setFramerate(&fpsm, 1.0 / timeStep);

    algorithm_drawingProcedure = drawingProcedure;

    return 0;

}


static void drawDIPS(SDL_Surface* surf, state* s, unsigned char r, unsigned char g, unsigned char b, unsigned char a, double ratioPixels) {

    double distance = (fabs(s->xPosition2 - s->xPosition1) * ratioPixels) - 14;
    double inter = (distance) / (double)NB_SPRING;
    unsigned int i = 1;
    int sign = 1;

    double x1 = (screenWidth / 4.0) + (s->xPosition1 * ratioPixels);
    double y1 = screenHeight / 2.0;
    double x2 = x1 - (ratioPixels * sin(M_PIl - s->angularPosition1) * 2.0 * parameters[2]);
    double y2 = y1 + (ratioPixels * cos(M_PIl - s->angularPosition1) * 2.0 * parameters[2]);

    aalineRGBA(surf, x1, y1, x2, y2, r, g, b, a);
    filledCircleRGBA(surf, x1, y1, 5, r, g, b, a);
    filledCircleRGBA(surf, x2, y2, 5, r, g, b, a);
    rectangleRGBA(surf, x1 - 7, y1 - 7, x1 + 7, y1 + 7, r, g, b, a);

    x1 = (screenWidth / 4.0) + (s->xPosition2 * ratioPixels);
    y1 = (screenHeight / 2.0);
    x2 = x1 - (ratioPixels * sin(M_PIl - s->angularPosition2) * 2.0 * parameters[3]);
    y2 = y1 + (ratioPixels * cos(M_PIl - s->angularPosition2) * 2.0 * parameters[3]);

    aalineRGBA(surf, x1, y1, x2, y2, r, g, b, a);
    filledCircleRGBA(surf, x1, y1, 5, r, g, b, a);
    filledCircleRGBA(surf, x2, y2, 5, r, g, b, a);
    rectangleRGBA(surf, x1 - 7, y1 - 7, x1 + 7, y1 + 7, r, g, b, a);

    x1 = (screenWidth / 4.0) + (s->xPosition1 * ratioPixels) + 7;
    y1 = (screenHeight / 2.0);
    for(; i <= NB_SPRING; i++) {
        if(sign > 0)
            sign = -7;
        else
            sign = 7;
        aalineRGBA(surf, x1+((i-1)*inter), y1+sign, x1+(i*inter), y1-sign, r, g, b, a);
    }

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
    double ratioPixels = ((screenWidth / 2.0) - 20) / (2.0 * parameters[1]);
    int done = 0;
    SDL_Event event;

    SDL_FillRect(screen, NULL, SDL_MapRGBA(screen->format, 255,255,255,255));

    if(a != NULL) {
        sprintf(str, "Applied X Force 1: % f", ((parameters[16] + parameters[16]) * a[0]) - parameters[16]);
        stringRGBA(screen, 5, 5, str, 0, 0, 0, 255);
        sprintf(str, "Applied X Force 2: % f", ((parameters[17] + parameters[17]) * a[1]) - parameters[17]);
        stringRGBA(screen, 5, 25, str, 0, 0, 0, 255);
    }

    sprintf(str, "Angular Position1: % f", s->angularPosition1);
    stringRGBA(screen, 5, 15, str, 0, 0, 0, 255);

    sprintf(str, "Angular Position2: % f", s->angularPosition2);
    stringRGBA(screen, 5, 35, str, 0, 0, 0, 255);

    sprintf(str, "Distance         : % f", fabs(s->xPosition2 - s->xPosition1));
    stringRGBA(screen, 5, 45, str, 0, 0, 0, 255);

    sprintf(str, "Reward           : % f", reward);
    stringRGBA(screen, 5, 55, str, 0, 0, 0, 255);

    lineRGBA(screen, 10, screenHeight / 2.0, screenHeight - 10, screenHeight / 2.0, 0, 0, 0, 255);

    if(algorithm_drawingProcedure != NULL)
        algorithm_drawingProcedure(screen, screenWidth, screenHeight, instance);

    drawDIPS(screen, s, 0, 0, 0, 255, ratioPixels);

    if(a != NULL) {
        if(a[1] < 0.5)
            filledTrigonRGBA(screen, ((screenWidth / 2.0) * 3.0 / 4.0) - 10, screenHeight * 0.9, ((screenWidth / 2.0) * 3.0 / 4.0) - 20, (screenHeight * 0.9) - 5, ((screenWidth / 2.0) * 3.0 / 4.0) - 10, (screenHeight * 0.9) - 10 , 0, 0, 0, 255);
        else if(a[1] > 0.5)
            filledTrigonRGBA(screen, ((screenWidth / 2.0) * 3.0 / 4.0) + 10, screenHeight * 0.9, ((screenWidth / 2.0) * 3.0 / 4.0) + 20, (screenHeight * 0.9) - 5, ((screenWidth / 2.0) * 3.0 / 4.0) + 10, (screenHeight * 0.9) - 10, 0, 0, 0, 255);
        else
            boxRGBA(screen, ((screenWidth / 2.0) * 3.0 / 4.0) - 5, screenHeight * 0.9, ((screenWidth / 2.0) * 3.0 / 4.0) + 5, (screenHeight * 0.9) - 10, 0, 0, 0, 255);

        if(a[0] < 0.5)
            filledTrigonRGBA(screen, (screenWidth / 8.0) - 10, screenHeight * 0.9, (screenWidth / 8.0) - 20, (screenHeight * 0.9) - 5, (screenWidth / 8.0) - 10, (screenHeight * 0.9) - 10 , 0, 0, 0, 255);
        else if(a[0] > 0.5)
            filledTrigonRGBA(screen, (screenWidth / 8.0) + 10, screenHeight * 0.9, (screenWidth / 8.0) + 20, (screenHeight * 0.9) - 5, (screenWidth / 8.0) + 10, (screenHeight * 0.9) - 10, 0, 0, 0, 255);
        else
            boxRGBA(screen, (screenWidth / 8.0) - 5, screenHeight * 0.9, (screenWidth / 8.0) + 5, (screenHeight * 0.9) - 10, 0, 0, 0, 255);
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

