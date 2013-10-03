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

#include "levitation.h"

unsigned int actionDimensionality = 1;          /* The number of dimension making up the action */
double timeStep = 0.004;                        /* Time step between two state */

double* parameters = NULL;                      /* Model's parameters */
unsigned int nbParameters = 11;                 /* Number of model's parameters */

/*+-----------Model's parameters----------+
  |                                       |
  | parameters[0] : mass of steel ball    |
  | parameters[1] : electrical resistance |
  | parameters[2] : coil parameter 1      |
  | parameters[3] : coil parameter 2      |
  | parameters[4] : coil parameter 3      |
  | parameters[5] : min process action    |
  | parameters[6] : max process action    |
  | parameters[7] : gravity               |
  | parameters[8] : min position          |
  | parameters[9] : max position          |
  | parameters[10]: goal position         |
  |                                       |
  +---------------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    gsl_rng* rng = gsl_rng_alloc(gsl_rng_mt19937);
    gsl_rng_set(rng, time(NULL));
    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 0.8;

    parameters[1] = 11.68;

    parameters[2] = 0.007;

    parameters[3] = 0.8052;

    parameters[4] = 0.001599;

    parameters[5] = -60;

    parameters[6] = 60;

    parameters[7] = 9.80665;

    parameters[8] = 0.000;

    parameters[9] = 0.013;

    parameters[10] = (gsl_rng_uniform(rng) * (parameters[9] - parameters[8])) + parameters[8];

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
    s->position = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->velocity = strtod(tmp, NULL);
    crt = end + 1;

    s->current = strtod(crt, NULL);

    return s;

}


/* Returns an allocated initial state of the model. */

state* initState() {

    double reward = 0;
    double a = (15.0 - parameters[5]) / (parameters[6] - parameters[5]) ;
    unsigned int i = 0;
    state* next = NULL;
    state* crt = (state*)malloc(sizeof(state));

    crt->position = parameters[9];
    crt->velocity = 0.0;
    crt->current = 0.0;

    for(; i < (0.5 / timeStep); i++) {
        nextStateReward(crt, &a, &next, &reward);
        free(crt);
        crt = next;
        next = NULL;
    }

    return crt;

}


double alpha(state* s) {

    return parameters[7] - (parameters[4] * s->current * s->current / (2.0 * parameters[0] * (parameters[2] + s->position) * (parameters[2] + s->position)));

}


double beta(state* s) {

    return s->current * ((parameters[4] * s->velocity) - (parameters[1] * (parameters[2] + s->position) * (parameters[2] + s->position))) / ((parameters[4] * (parameters[2] + s->position)) + (parameters[3] * (parameters[2] + s->position) * (parameters[2] + s->position)));

}


double gamma(state* s) {

    return (parameters[2] + s->position) / (parameters[4] + (parameters[3] * (parameters[2] + s->position)));

}


void RK4OneStep(state* sIn, state* sOut, double h, double u) {

    state tmp;
    state k1;
    state k2;
    state k3;
    state k4;

    k1.position = sIn->velocity;
    k1.velocity = alpha(sIn);
    k1.current = beta(sIn) + (gamma(sIn) * u);

    tmp.position = sIn->position + (k1.position * (h / 2.0));
    tmp.velocity = sIn->velocity + (k1.velocity * (h / 2.0));
    tmp.current = sIn->current + (k1.current * (h / 2.0));

    k2.position = tmp.velocity;
    k2.velocity = alpha(&tmp);
    k2.current = beta(&tmp) + (gamma(&tmp) * u);

    tmp.position = sIn->position + (k2.position * (h / 2.0));
    tmp.velocity = sIn->velocity + (k2.velocity * (h / 2.0));
    tmp.current = sIn->current + (k2.current * (h / 2.0));

    k3.position = tmp.velocity;
    k3.velocity = alpha(&tmp);
    k3.current = beta(&tmp) + (gamma(&tmp) * u);

    tmp.position = sIn->position + (k3.position * h);
    tmp.velocity = sIn->velocity + (k3.velocity * h);
    tmp.current = sIn->current + (k3.current * h);

    k4.position = tmp.velocity;
    k4.velocity = alpha(&tmp);
    k4.current = beta(&tmp) + (gamma(&tmp) * u);

    sOut->position = sIn->position + (h * (k1.position + (2.0 * k2.position) + (2.0 * k3.position) + k4.position) / 6.0);
    sOut->velocity = sIn->velocity + (h * (k1.velocity + (2.0 * k2.velocity) + (2.0 * k3.velocity) + k4.velocity) / 6.0);
    sOut->current = sIn->current + (h * (k1.current + (2.0 * k2.current) + (2.0 * k3.current) + k4.current) / 6.0);

    if(sOut->position > parameters[9]) {
        sOut->position = parameters[9];
        sOut->velocity = 0.0;
    } else if(sOut->position < parameters[8]) {
        sOut->position = parameters[8];
        sOut->velocity = 0.0;
    }

}


char nextStateReward(state* s, double* a, state** nextState,double* reward) {

    double realAction = (*a * (parameters[6] - parameters[5])) + parameters[5];
    *nextState = (state*)malloc(sizeof(state));

    RK4OneStep(s, *nextState, timeStep / 3.0, realAction);
    RK4OneStep(*nextState, *nextState, timeStep / 3.0, realAction);
    RK4OneStep(*nextState, *nextState, timeStep / 3.0, realAction);

    *reward = 1.0 - (fabs((*nextState)->position - parameters[10]) / (parameters[9] - parameters[8]));

    return 0;

}


/* Returns an allocated copy of the state s */

state* copyState(state* s) {

    state* newState = (state*)malloc(sizeof(state));
    memcpy(newState, s, sizeof(state));

    return newState;

}


void printState(state* s) {

    printf("position: % 2.5f velocity: % 2.5f current: % 2.5f ",s->position, s->velocity, s->current);

}


void printAction(double* a) {

    printf("action: % 2.5f ", ((parameters[6] - parameters[5]) * *a) + parameters[5]);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
