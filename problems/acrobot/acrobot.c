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

#include <stdio.h>
#include <stdlib.h>
#define __USE_GNU
#include <math.h>
#undef __USE_GNU
#include <string.h>

#include "acrobot.h"

#ifndef NUMBER_OF_DIMENSIONS_OF_ACTION
    #error "You should add -D NUMBER_OF_DIMENSIONS_OF_ACTION=1 to your compiler arguments"
#endif

double timeStep = 0.1;                          /* Time step between two state */

double* parameters = NULL;                      /* Model's parameters */
unsigned int nbParameters = 9;                  /* Number of model's parameters */

/*+-----------------Model's parameters----------------+
  |                                                   |
  | parameters[0] : lenght of the first link          |
  | parameters[1] : mass of the first link            |
  | parameters[2] : friction of the first link        |
  | parameters[3] : lenght of the second link         |
  | parameters[4] : mass of the second link           |
  | parameters[5] : friction of the second link       |
  | parameters[6] : gravity                           |
  | parameters[7] : torque applied to the second link |
  | parameters[8] : max angular velocity              |
  |                                                   |
  +---------------------------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 0.5;

    parameters[1] = 1.0;

    parameters[2] = 0.05;

    parameters[3] = 0.5;

    parameters[4] = 1.0;

    parameters[5] = 0.05;

    parameters[6] = 9.81;

    parameters[7] = 2.0;

    parameters[8] = 15.0;

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
    s->angularPosition1 = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->angularVelocity1 = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->angularPosition2 = strtod(tmp, NULL);
    crt = end + 1;

    s->angularVelocity2 = strtod(crt, NULL);

    s->isTerminal = 0;

    return s;


}

/* Returns an allocated initial state of the model. */

state* initState() {

    state* init = (state*)malloc(sizeof(state));

    init->angularPosition1 = M_PIl;

    init->angularVelocity1 = 0.0;

    init->angularPosition2 = M_PIl;

    init->angularVelocity2 = 0.0;

    init->isTerminal = 0;

    return init;

}


/* Returns a triplet containing the next state, the applied action and the reward given the current state and action. */

char nextStateReward(state* s, double* a, state** nextState,double* reward) {

    *nextState = copyState(s);

    if(s->isTerminal < 0) {
        *reward = 0.0;
    } else {
        double x = 0.0;
        double y = 0.0;

        double m1 = parameters[1];
        double l1 = parameters[0];
        double mu1 = parameters[2];
        double m2 = parameters[4];
        double l2 = parameters[3];
        double mu2 = parameters[5];

        double a11 = ((4.0 / 3.0) * m1 + 4 * m2) * l1 * l1;
        double a22 = (4.0 / 3.0) * m2 * l2 * l2;
        double m2l2l12 = 2 * m2 * l1 * l2;
        double coef1 = (m1 + 2 * m2) * l1 * 9.81;
        double coef2 = m2 * l2 * 9.81;

        double a12 = m2l2l12 * cos((*nextState)->angularPosition2 - (*nextState)->angularPosition1);
        double Det = a11 * a22 - a12 * a12;

        double s = sin((*nextState)->angularPosition2 - (*nextState)->angularPosition1);
        double appliedTorque = ((parameters[7] + parameters[7]) * a[0]) - parameters[7];
        double b1 = coef1 * sin((*nextState)->angularPosition1) + m2l2l12 * (*nextState)->angularVelocity2 * (*nextState)->angularVelocity2 * s - appliedTorque - mu1 * (*nextState)->angularVelocity1;
        double b2 = coef2 * sin((*nextState)->angularPosition2) - m2l2l12 * (*nextState)->angularVelocity1 * (*nextState)->angularVelocity1 * s + appliedTorque - mu2 * (*nextState)->angularVelocity2;

        (*nextState)->angularPosition1 += (*nextState)->angularVelocity1 * timeStep;  
        (*nextState)->angularPosition2 += (*nextState)->angularVelocity2 * timeStep;
        (*nextState)->angularVelocity1 += ((a22 * b1 - a12 * b2) / Det) * timeStep;
        (*nextState)->angularVelocity2 += ((-a12 * b1 + a11 * b2) / Det) * timeStep;

        if((*nextState)->angularVelocity1 > parameters[8])
            (*nextState)->angularVelocity1 = parameters[8];

        if((*nextState)->angularVelocity1 < -parameters[8])
            (*nextState)->angularVelocity1 = -parameters[8];

        if((*nextState)->angularVelocity2 > parameters[8])
            (*nextState)->angularVelocity2 = parameters[8];

        if((*nextState)->angularVelocity2 < -parameters[8])
            (*nextState)->angularVelocity2 = -parameters[8];


        if((*nextState)->angularPosition1 > (2.0 * M_PIl))
            (*nextState)->angularPosition1 -= 2.0 * M_PIl;

        if((*nextState)->angularPosition1 < 0.0)
            (*nextState)->angularPosition1 += 2.0 * M_PIl;

        if((*nextState)->angularPosition2 > (2.0 * M_PIl))
            (*nextState)->angularPosition2 -= 2.0 * M_PIl;

        if((*nextState)->angularPosition2 < 0.0)
            (*nextState)->angularPosition2 += 2.0 * M_PIl;

        x = (sin((*nextState)->angularPosition1) * l1) + (sin((*nextState)->angularPosition2) * l2);
        y = (cos((*nextState)->angularPosition1) * l1) + (cos((*nextState)->angularPosition2) * l2);

        *reward = 1.0 - (sqrt(((y - (l1 + l2)) * (y - (l1 + l2))) + (x * x)) / (2.0 * (l1 + l2)));

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

    printf("angularPosition1: % f angularVelocity1: % f angularPosition2: % f  angularVelocity2: % f ",s->angularPosition1, s->angularVelocity1, s->angularPosition2, s->angularVelocity2);

}


void printAction(double* a) {

    printf("action: % f ", ((parameters[7] + parameters[7]) * a[0]) - parameters[7]);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
