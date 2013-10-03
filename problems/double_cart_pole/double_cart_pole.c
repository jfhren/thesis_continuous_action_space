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

#include "double_cart_pole.h"

#ifndef NUMBER_OF_DIMENSIONS_OF_ACTION
    #error "You should add -D NUMBER_OF_DIMENSIONS_OF_ACTION=2 to your compiler arguments"
#endif

double timeStep = 0.1;                          /* Time step between two state */

double* parameters = NULL;                      /* Model's parameters */
unsigned int nbParameters = 22;                 /* Number of model's parameters */


/*+----------------------Model's parameters----------------------+
  |                                                              |
  | parameters[0]  : gravity                                     |
  | parameters[1]  : half-lenght of the track                    |
  | parameters[2]  : half-lenght of the first pole               |
  | parameters[3]  : half-lenght of the second pole              |
  | parameters[4]  : mass of the first cart                      |
  | parameters[5]  : mass of the second cart                     |
  | parameters[6]  : mass of the first pole                      |
  | parameters[7]  : mass of the second pole                     |
  | parameters[8]  : friction coefficient of the first cart      |
  | parameters[9]  : friction coefficient of the second cart     |
  | parameters[10] : friction coefficient of the first pole      |
  | parameters[11] : friction coefficient of the second pole     |
  | parameters[12] : coefficient K of the spring                 |
  | parameters[13] : relaxed lenght of the spring                |
  | parameters[14] : minimum lenght of the spring                |
  | parameters[15] : maximum lenght of the spring                |
  | parameters[16] : maximum acceleration of the first pole      |
  | parameters[17] : maximum acceleration of the second pole     |
  | parameters[18] : maximum velocity of the first cart          |
  | parameters[19] : maximum velocity of the second cart         |
  | parameters[20] : maximum angular velocity of the first pole  |
  | parameters[21] : maximum angular velocity of the second pole |
  |														 		 |
  +--------------------------------------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/
void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 9.81;
    parameters[1] = 2.4;
    parameters[2] = 0.5;
    parameters[3] = 0.5;
    parameters[4] = 1.0;
    parameters[5] = 1.0;
    parameters[6] = 0.1;
    parameters[7] = 0.1;
    parameters[8] = 0.0005;
    parameters[9] = 0.0005;
    parameters[10] = 0.000002;
    parameters[11] = 0.000002;
    parameters[12] = 2.0;
    parameters[13] = 0.5;
    parameters[14] = 0.2;
    parameters[15] = 1.0;
    parameters[16] = 5.0;
    parameters[17] = 5.0;
    parameters[18] = 15.0;
    parameters[19] = 15.0;
    parameters[20] = 10.0;
    parameters[21] = 10.0;

}


/* Initialisation of the generative model. To call after parameters initialisation. */
void initGenerativeModel(){

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
    s->xPosition1 = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->xVelocity1 = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
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
    s->xPosition2 = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->xVelocity2 = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->angularPosition2 = strtod(tmp, NULL);
    crt = end + 1;

    s->angularVelocity2 = strtod(crt, NULL);

    if((s->xPosition2 <= s->xPosition1) || (fabs(s->xPosition2 - s->xPosition1) < parameters[14]) || (fabs(s->xPosition2 - s->xPosition1) > parameters[15]))
        s->xPosition2 = s->xPosition1 + parameters[14] + 0.01;

    if((s->xVelocity1 > 0.0) && (s->xVelocity2 < 0.0))
        s->xVelocity2 = s->xVelocity1;

    s->isTerminal = 0;

    return s;

}


/* Returns an allocated initial state of the model. */

state* initState() {

    state* initial = (state*)malloc(sizeof(state));

    initial->xPosition1 = 0.0;
    initial->angularPosition1 = M_PIl;
    initial->xVelocity1 = 0.0;
    initial->angularVelocity1 = 0.0;

    initial->xPosition2 = initial->xPosition1 + parameters[14] + 0.01;
    initial->angularPosition2 = M_PIl;
    initial->xVelocity2 = 0.0;
    initial->angularVelocity2 = 0.0;

    initial->isTerminal = 0;

    return initial;

}


char nextStateReward(state* s, double* a, state** nextState,double* reward) {

    *nextState = copyState(s);
    *reward = 0.0;

    if(!(*nextState)->isTerminal) {
        double a11_1 = (4.0 * parameters[2]) / 3.0;
        double a22_1 = -(parameters[4] + parameters[6]);
        double a11_2 = (4.0 * parameters[3]) / 3.0;
        double a22_2 = -(parameters[5] + parameters[7]);
        double xForce = (((parameters[16] + parameters[16]) * a[0]) - parameters[16]) - (parameters[12] * (parameters[13] - fabs((*nextState)->xPosition2 - (*nextState)->xPosition1)));
        double a12 = -cos((*nextState)->angularPosition1);
        double a21 = parameters[2] * parameters[6] * cos((*nextState)->angularPosition1);
        double b1 = parameters[0] * sin((*nextState)->angularPosition1) - ((parameters[10] * (*nextState)->angularVelocity1) / (parameters[2] * parameters[6]));
        double b2 = (parameters[2] * parameters[6] * (*nextState)->angularVelocity1 * (*nextState)->angularVelocity1 * sin((*nextState)->angularPosition1)) - xForce + ((*nextState)->xVelocity1 > 0.0 ? -parameters[8] : parameters[8]);

        double angularAcceleration1 = ((b2 * a12) - (a22_1 * b1)) / ((a12 * a21) - (a11_1 * a22_1));
        double xAcceleration1 = (b1 - (a11_1 * angularAcceleration1)) / a12;

        (*nextState)->angularVelocity1 = (*nextState)->angularVelocity1 + (timeStep * angularAcceleration1);
        if(fabs((*nextState)->angularVelocity1) > parameters[20])
            (*nextState)->angularVelocity1 = (*nextState)->angularVelocity1 > 0.0 ? parameters[20] : - parameters[20];

        (*nextState)->xVelocity1 = (*nextState)->xVelocity1 + (timeStep * xAcceleration1);
        if(fabs((*nextState)->xVelocity1) > parameters[18])
            (*nextState)->xVelocity1 = (*nextState)->xVelocity1 > 0.0 ? parameters[18] : - parameters[18];

        (*nextState)->angularPosition1 = (*nextState)->angularPosition1 + (timeStep * (*nextState)->angularVelocity1);
        if((*nextState)->angularPosition1 > (2.0 * M_PIl))
            (*nextState)->angularPosition1 = (*nextState)->angularPosition1 - (2.0 * M_PIl);
        if((*nextState)->angularPosition1 < 0.0)
            (*nextState)->angularPosition1 = (*nextState)->angularPosition1 + (2.0 * M_PIl);

        (*nextState)->xPosition1 = (*nextState)->xPosition1 + (timeStep * (*nextState)->xVelocity1);


        xForce = (((parameters[17] + parameters[17]) * a[1]) - parameters[17]) + (parameters[12] * (parameters[13] - fabs((*nextState)->xPosition2 - (*nextState)->xPosition1)));
        a12 = -cos((*nextState)->angularPosition2);
        a21 = parameters[3] * parameters[7] * cos((*nextState)->angularPosition2);
        b1 = parameters[0] * sin((*nextState)->angularPosition2) - ((parameters[11] * (*nextState)->angularVelocity2) / (parameters[3] * parameters[7]));
        b2 = (parameters[3] * parameters[7] * (*nextState)->angularVelocity2 * (*nextState)->angularVelocity2 * sin((*nextState)->angularPosition2)) - xForce + ((*nextState)->xVelocity2 > 0.0 ? -parameters[9] : parameters[9]);

        double angularAcceleration2 = ((b2 * a12) - (a22_2 * b1)) / ((a12 * a21) - (a11_2 * a22_2));
        double xAcceleration2 = (b1 - (a11_2 * angularAcceleration2)) / a12;

        (*nextState)->angularVelocity2 = (*nextState)->angularVelocity2 + (timeStep * angularAcceleration2);
        if(fabs((*nextState)->angularVelocity2) > parameters[20])
            (*nextState)->angularVelocity2 = (*nextState)->angularVelocity2 > 0.0 ? parameters[20] : - parameters[20];

        (*nextState)->xVelocity2 = (*nextState)->xVelocity2 + (timeStep * xAcceleration2);
        if(fabs((*nextState)->xVelocity2) > parameters[18])
            (*nextState)->xVelocity2 = (*nextState)->xVelocity2 > 0.0 ? parameters[18] : - parameters[18];

        (*nextState)->angularPosition2 = (*nextState)->angularPosition2 + (timeStep * (*nextState)->angularVelocity2);
        if((*nextState)->angularPosition2 > (2.0 * M_PIl))
            (*nextState)->angularPosition2 = (*nextState)->angularPosition2 - (2.0 * M_PIl);
        if((*nextState)->angularPosition2 < 0.0)
            (*nextState)->angularPosition2 = (*nextState)->angularPosition2 + (2.0 * M_PIl);

        (*nextState)->xPosition2 = (*nextState)->xPosition2 + (timeStep * (*nextState)->xVelocity2);

        if((fabs((*nextState)->xPosition1) >= parameters[1]) || (fabs((*nextState)->xPosition2) >= parameters[1]) || ((*nextState)->xPosition2 <= (*nextState)->xPosition1) || (fabs((*nextState)->xPosition2 - (*nextState)->xPosition1) < parameters[14]) || (fabs((*nextState)->xPosition2 - (*nextState)->xPosition1) > parameters[15]))
            (*nextState)->isTerminal = -1;
        else
            *reward = ((1.0 + cos((*nextState)->angularPosition1)) / 4.0) + ((1.0 + cos((*nextState)->angularPosition2)) / 4.0);
    }

    return (*nextState)->isTerminal;

}


/* Returns an allocated copy of the state s */

state* copyState(state* s) {

    state* nextState = (state*)malloc(sizeof(state));
    memcpy(nextState, s, sizeof(state));

    return nextState;

}


void printState(state* s) {

    printf("xPosition1: %f angularPosition1: %f xPosition2: %f angularPosition2: %f ", s->xPosition1, s->angularPosition1, s->xPosition2, s->angularPosition2);

}


void printAction(double* a) {

    printf("action: (%f,%f) ", ((parameters[16] + parameters[16]) * a[0]) - parameters[16], ((parameters[17] + parameters[17]) * a[1]) - parameters[17]);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
