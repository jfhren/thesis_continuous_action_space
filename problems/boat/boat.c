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
#include <gsl/gsl_rng.h>
#include <time.h>

#include "boat.h"

#ifndef NUMBER_OF_DIMENSIONS_OF_ACTION
    #error "You should add -D NUMBER_OF_DIMENSIONS_OF_ACTION=1 to your compiler arguments"
#endif

double timeStep = 0.1;							/* Time step between two state */

double* parameters = NULL;						/* Model's parameters */
unsigned int nbParameters = 10;					/* Number of model's parameters */

/*+------------Model's parameters----------+
  |                                        |
  | parameters[0]: force of the current    |
  | parameters[1]: system inertia          |
  | parameters[2]: maximum speed           |
  | parameters[3]: speed goal              |
  | parameters[4]: rudder coefficient      |
  | parameters[5]: quay x position         |
  | parameters[6]: quay y position         |
  | parameters[7]: quay half width         |
  | parameters[8]: minimum input direction |
  | parameters[9]: maximum input direction |
  |                                        |
  +----------------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 1.25;

    parameters[1] = 0.1;

    parameters[2] = 2.5;

    parameters[3] = 1.75;

    parameters[4] = 0.9;

    parameters[5] = 200;

    parameters[6] = 110;

    parameters[7] = 10;

    parameters[8] = -M_PIl / 2.0;

    parameters[9] = M_PIl / 2.0;

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
    s->yPosition = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->boatAngle = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->rudderAngle = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->velocity = strtod(tmp, NULL);
    crt = end + 1;

    s->omega = strtod(crt, NULL);

    s->isTerminal = 0;

    return s;

}


/* Returns an allocated initial state of the model. */

state* initState() {
    gsl_rng* rng = gsl_rng_alloc(gsl_rng_mt19937);
    gsl_rng_set(rng, time(NULL));

    state* init = (state*)malloc(sizeof(state));

    init->xPosition = 0.0;
    init->yPosition = gsl_rng_uniform(rng) * 200;

    init->boatAngle = (gsl_rng_uniform(rng) * M_PIl) - (M_PIl/2.0);
    init->rudderAngle = 0.0;

    init->velocity = 0.0;
    init->omega = 0.0;
    init->isTerminal = 0;

    return init;

}


char nextStateReward(state* s, double* a, state** nextState,double* reward) {

    if(s->isTerminal) {	
        *nextState = copyState(s);
        *reward = s->isTerminal < 0 ? 0.0 : 1.0;
    } else {
        double quarterPI = M_PIl / 4.0;
        double distance = 0;

        *nextState = (state*)malloc(sizeof(state));

        (*nextState)->rudderAngle = parameters[4] * (((*a * (parameters[9] - parameters[8])) + parameters[8]) - s->boatAngle);
        if((*nextState)->rudderAngle < -quarterPI)
            (*nextState)->rudderAngle = -quarterPI;
        else if((*nextState)->rudderAngle > quarterPI)
            (*nextState)->rudderAngle = quarterPI;

        (*nextState)->velocity = s->velocity + ((parameters[3] - s->velocity) * parameters[1]);
        (*nextState)->omega = s->omega + (((*nextState)->rudderAngle - s->omega) * ((*nextState)->velocity / parameters[2]));
        (*nextState)->boatAngle = s->boatAngle + (parameters[1] * (*nextState)->omega);
        (*nextState)->xPosition = s->xPosition + ((*nextState)->velocity * cos((*nextState)->boatAngle));
        if((*nextState)->xPosition < 0)
            (*nextState)->xPosition = 0;
        else if((*nextState)->xPosition > 200)
            (*nextState)->xPosition = 200;

        (*nextState)->yPosition = s->yPosition - ((*nextState)->velocity * sin((*nextState)->boatAngle)) - (parameters[0] * (((*nextState)->xPosition / 50.0) - ((*nextState)->xPosition * (*nextState)->xPosition / 10000.0)));
        if((*nextState)->yPosition < 0)
            (*nextState)->yPosition = 0;
        else if((*nextState)->yPosition > 200)
            (*nextState)->yPosition = 200;

        distance = sqrt(((parameters[5] - (*nextState)->xPosition) * (parameters[5] - (*nextState)->xPosition)) + ((parameters[6] - (*nextState)->yPosition) * (parameters[6] - (*nextState)->yPosition)));

        if(((*nextState)->xPosition == parameters[5]) && (distance > parameters[7])) {
            (*nextState)->isTerminal = -1;
            *reward = 0.0;
        } else if(((*nextState)->xPosition == parameters[5]) && (distance < parameters[7])) {
            (*nextState)->isTerminal = 1;
            *reward = 1.0;
        } else {
            (*nextState)->isTerminal = 0;
            *reward = 1.0 - (distance / sqrt((200 * 200) + (200 * 200)));
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

    printf("xPosition: % 2.5f yPosition: % 2.5f rudderAngle: % 2.5f boatAngle: % 2.5f ",s->xPosition, s->yPosition, s->rudderAngle, s->boatAngle);

}


void printAction(double* a) {

    printf("action: % 2.5f ", ((parameters[9] - parameters[8]) * *a) + parameters[8]);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
