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

#include <stdlib.h>
#include <stdio.h>
#define __USE_GNU
#include <math.h>
#undef __USE_GNU
#include <string.h>

#include "cart_pole.h"

#ifndef NUMBER_OF_DIMENSIONS_OF_ACTION
    #error "You should add -D NUMBER_OF_DIMENSIONS_OF_ACTION=1 to your compiler arguments"
#endif

double timeStep = 0.1;							/* Time step between two state */

double* parameters = NULL;						/* Model's parameters */
unsigned int nbParameters = 10;					/* Number of model's parameters */

/*+---------------Model's parameters--------------+
  |                                               |
  | parameters[0]: gravity                        |
  | parameters[1]: half-lenght of the track       |
  | parameters[2]: half-lenght of a pole          |
  | parameters[3]: mass of a cart                 |
  | parameters[4]: mass of a pole                 |
  | parameters[5]: friction coefficient of a cart |
  | parameters[6]: friction coefficient of a pole |
  | parameters[7]: maximum acceleration           |
  | parameters[8]: maximum velocity               |
  | parameters[9]: maximum angular velocity       |
  |                                               |
  +-----------------------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 9.81;

    parameters[1] = 2.4;

    parameters[2] = 0.5;

    parameters[3] = 1.0;

    parameters[4] = 0.1;

    parameters[5] = 0.0005;

    parameters[6] = 0.000002;

    parameters[7] = 10.0;

    parameters[8] = 15.0;

    parameters[9] = 10.0;

}


/* Initialisation of the generative model. To call after parameters initialisation. */

void initGenerativeModel() {

    return;

}


/* Free the generative model. To call if the generative model has to be discarded. */

void freeGenerativeModel() {

    return;

}


/* Free the generative model parameters. To call afer everything is finished. */

void freeGenerativeModelParameters() {

    free(parameters);

}


/* Returns an allocated state initialized from the parsed string */
state* makeState(const char* str) {

    char* end = NULL;
    char* crt = (char*)str;
    char tmp[255];    
    state* s = (state*)malloc(sizeof(state));

    end = strchr(str, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->xPosition = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->xVelocity = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->angularPosition = strtod(tmp, NULL);
    crt = end + 1;

    s->angularVelocity = strtod(crt, NULL);

    s->isTerminal = 0;

    return s;

}


/* Returns an allocated initial state of the model. */

state* initState() {

    state* init = (state*)malloc(sizeof(state));

    init->xPosition = 0.0;
    init->xVelocity = 0.0;

    init->angularPosition = M_PIl;
    init->angularVelocity = 0.0;

    init->isTerminal = 0;

    return init;

}


char nextStateReward(state* s, double* a, state** nextState,double* reward) {

    if(s->isTerminal) {	
        *nextState = copyState(s);
        *reward = 0.0;
    } else {
        double a11 = (4.0 * parameters[2]) / 3.0;
        double a22 = -(parameters[3] + parameters[4]);
        double a12 = -cos(s->angularPosition);
        double a21 = parameters[2] * parameters[4] * cos(s->angularPosition);
        double b1 = parameters[0] * sin(s->angularPosition) - ((parameters[6] * s->angularVelocity) / (parameters[2] * parameters[4]));
        double b2 = (parameters[2] * parameters[4] * s->angularVelocity * s->angularVelocity * sin(s->angularPosition)) - (((parameters[7] + parameters[7]) * a[0]) - parameters[7]) + (s->xVelocity == 0 ? 0: (s->xVelocity > 0.0 ? -parameters[5] : parameters[5]));
        double angularAcceleration = ((b2 * a12) - (a22 * b1)) / ((a12 * a21) - (a11 * a22));
        double xAcceleration = (b1 - (a11 * angularAcceleration)) / a12;

        *nextState = (state*)malloc(sizeof(state));

        (*nextState)->angularVelocity = s->angularVelocity + (timeStep * angularAcceleration);
        if(fabs((*nextState)->angularVelocity) > parameters[9])
            (*nextState)->angularVelocity = (*nextState)->angularVelocity > 0.0 ? parameters[9] : - parameters[9];

        (*nextState)->xVelocity = s->xVelocity + (timeStep * xAcceleration);
        if(fabs((*nextState)->xVelocity) > parameters[8])
            (*nextState)->xVelocity = (*nextState)->xVelocity > 0.0 ? parameters[8] : - parameters[8];

        (*nextState)->angularPosition = s->angularPosition + (timeStep * (*nextState)->angularVelocity);
        if((*nextState)->angularPosition > (2.0 * M_PIl))
            (*nextState)->angularPosition = (*nextState)->angularPosition - (2.0 * M_PIl);
        if((*nextState)->angularPosition < 0.0)
            (*nextState)->angularPosition = (*nextState)->angularPosition + (2.0 * M_PIl);

        (*nextState)->xPosition = s->xPosition + (timeStep * (*nextState)->xVelocity);

        if(fabs((*nextState)->xPosition) > parameters[1]) {
            (*nextState)->isTerminal = -1;
            *reward = 0.0;
        } else {
            (*nextState)->isTerminal = 0;
            *reward = (1.0 + cos((*nextState)->angularPosition)) / 2.0;
        }
    }

    return (*nextState)->isTerminal;

}


/* Returns an allocated copy of the state s */

state* copyState(state* s) {

    state* newState = (state*)malloc(sizeof(state));
    memcpy(newState, s, sizeof(state));

    return newState;

}


void printState(state* s) {

    printf("xPosition: % 2.5f angularPosition: % 2.5f ",s->xPosition, s->angularPosition);

}


void printAction(double* a) {

    printf("action: % 2.5f ", ((parameters[7] + parameters[7]) * a[0]) - parameters[7]);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
