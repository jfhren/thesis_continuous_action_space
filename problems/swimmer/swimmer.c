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
#include <gsl/gsl_linalg.h>

#include "swimmer.h"

unsigned int actionDimensionality = NUMBER_OF_DIMENSIONS_OF_ACTION; /* The number of dimension making up the action */
double timeStep = 0.02;                          /* Time step between two state */

double* parameters = NULL;                      /* Model's parameters */
unsigned int nbParameters = 7;                  /* Number of model's parameters */

static int segments = NUMBER_OF_DIMENSIONS_OF_ACTION + 1;

/*+---------------Model's parameters--------------+
  |                                               |
  | parameters[0] : lenght of a body part         |
  | parameters[1] : mass of a body part           |
  | parameters[2] : viscous-friction coefficient  |
  | parameters[3] : torque applied to a body part |
  | parameters[4] : x goal                        |
  | parameters[5] : y goal                        |
  | parameters[6] : size of the field             |
  |                                               |
  +-----------------------------------------------+*/

static double pdMatrix[((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 2) * ((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3)];       // (n+2)*(n+3) system: x y Theta_1 ... Theta_n
static double pdLU[((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 2) * ((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 2)];           // (n+2)*(n+2) LU decomposition matrix
static double pdb[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 2];                                                           // (n+2) constant right-hand vector
static double pdRH[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 2];                                                          // (n+2) right-hand when solving linear system
static double pdSolution[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 2];                                                    // (n+2) linear system solution
static double pdADotx[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 1];                                                       // (n+1) Array of x coordinate of joint velocity
static double pdADoty[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 1];                                                       // (n+1) Array of y corrdinate of joint velocity
static double pdGDotDotx[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3];                                                    // (n+3) G acceleration (x)
static double pdGDotDoty[(NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3];                                                    // (n+3) G acceleration (y)
static double pdADotDotx[((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 1) * ((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3)];     // (n+1)*(n+3) equations of joint acceleration (x)
static double pdADotDoty[((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 1) * ((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3)];     // (n+1)*(n+3) equations of joint acceleration (y)
static double pdfx[((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 1) * ((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3)];           // (n+1)*(n+3) equations for internal forces (x)
static double pdfy[((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 1) * ((NUMBER_OF_DIMENSIONS_OF_ACTION + 1) + 3)];           // (n+1)*(n+3) equations for internal forces (y)

/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 1.0;

    parameters[1] = 1.0;

    parameters[2] = 10.0;

    parameters[3] = 5.0;

    parameters[4] = 2.5;

    parameters[5] = 4.0;

    parameters[6] = 5.0;

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


static void getG(state* s, double* Gx, double* Gy) {

    double x = 0.0;
    double y = 0.0;
    int i = 0;
    *Gx = 0.0;
    *Gy = 0.0;


    for(; i < segments; i++) {
        *Gx += x;
        *Gy += y;
        x += cos(s->theta[i]);
        y += sin(s->theta[i]);
        *Gx += x;
        *Gy += y;
    }

    *Gx /= 2 * segments;
    *Gy /= 2 * segments;

}



/* Returns an allocated state initialized from the parsed string */
state* makeState(const char* str) {

    char* end = NULL;
    char* crt = (char*)str;
    double G[2] = {0.0,0.0};
    char tmp[255];
    int i = 0;
    state* s = (state*)malloc(sizeof(state));

    end = strchr(str, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->AZero[0] = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->AZero[1] = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->GDot[0] = strtod(tmp, NULL);
    crt = end + 1;

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->GDot[1] = strtod(tmp, NULL);
    crt = end + 1;


    for(; i < (segments - 1); i++) {
        end = strchr(crt, ',');
        memcpy(tmp, crt, end - crt);
        tmp[end-crt] = '\0';
        s->theta[i] = strtod(tmp, NULL);
        crt = end + 1;

        end = strchr(crt, ',');
        memcpy(tmp, crt, end - crt);
        tmp[end-crt] = '\0';
        s->thetaDot[i] = strtod(tmp, NULL);
        crt = end + 1;
    }

    end = strchr(crt, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->theta[i] = strtod(tmp, NULL);
    crt = end + 1;

    s->thetaDot[i] = strtod(crt, NULL);

    getG(s, G, G + 1);
    s->G[0] = G[0] + s->AZero[0];
    s->G[1] = G[1] + s->AZero[1];

    for(i = 0; i < 2; i++) {
        if(s->G[i] < 0.0) {
            s->G[i] = 0.0;
            s->AZero[i] = -G[i];
        } else if (s->G[i] > parameters[6]){
            s->G[i] = parameters[6];
            s->AZero[i] = parameters[6] - G[i];
        }
    }   

    s->isTerminal = 0;

    return s;

}

/* Returns an allocated initial state of the model. */

state* initState() {

    int i = 0;
    state* init = (state*)malloc(sizeof(state));

    init->AZero[0] = 0.0;
    init->AZero[1] = 0.0;

    init->GDot[0] = 0.0;
    init->GDot[1] = 0.0;

    for(;i < segments; i++) {
        init->theta[i] = 0.0;
        init->thetaDot[i] = 0.0;
    }

    getG(init, init->G, init->G + 1);
    init->G[0] += init->AZero[0];
    init->G[1] += init->AZero[1];

    init->isTerminal = 0;

    return init;

}


static void computeMatrix(state *x) {

    int i = 0;
    const double coeff = 0.5 * parameters[1] / (segments * parameters[1]);
    double GDotx = 0.0;
    double GDoty = 0.0;

    for (i = segments + 3; --i >= 0;) {
        pdfx[i] = 0.0;
        pdfy[i] = 0.0;
    }

/*+----------------+
  |ADot and ADotDot|
  +----------------+*/

    pdADotx[0] = 0.0;
    pdADoty[0] = 0.0;

    for (i = segments + 3; --i >= 0;){
        pdADotDotx[i] = 0.0;
        pdADotDoty[i] = 0.0;
        pdGDotDotx[i] = 0.0;
        pdGDotDoty[i] = 0.0;
    }

    for (i = 1; i <= segments; i++) {
        int j = segments + 3;
        double c = cos(x->theta[i - 1]);
        double s = sin(x->theta[i - 1]);
        double thetaDot = x->thetaDot[i - 1];
        const int line = i * (segments + 3);
        const int prevLine = (i - 1) * (segments + 3);

        pdADotx[i] = pdADotx[i - 1] - parameters[0] * thetaDot * s;
        pdADoty[i] = pdADoty[i - 1] + parameters[0] * thetaDot * c;

        for (; --j >= 0;) {
            pdADotDotx[line + j] = pdADotDotx[prevLine + j];
            pdADotDoty[line + j] = pdADotDoty[prevLine + j];
        }
        pdADotDotx[line + i + 1] += -parameters[0] * s;
        pdADotDoty[line + i + 1] += parameters[0] * c;
        pdADotDotx[line + segments + 2] += parameters[0] * thetaDot * thetaDot * c;
        pdADotDoty[line + segments + 2] += parameters[0] * thetaDot * thetaDot * s;

        GDotx += coeff * (pdADotx[i - 1] + pdADotx[i]);
        GDoty += coeff * (pdADoty[i - 1] + pdADoty[i]);
        for (j = segments + 3; --j >= 0;) {
            pdGDotDotx[j] += coeff * (pdADotDotx[prevLine + j] + pdADotDotx[line + j]);
            pdGDotDoty[j] += coeff * (pdADotDoty[prevLine + j] + pdADotDoty[line + j]);
        }
    }

    for (i = 0; i <= segments; i++) {
        int j = segments + 3;
        pdADotx[i] += x->GDot[0] - GDotx;
        pdADoty[i] += x->GDot[1] - GDoty;
        pdADotDotx[i * (segments + 3) + 0] += 1.0;
        pdADotDoty[i * (segments + 3) + 1] += 1.0;
        for (; --j >= 0;) {
            pdADotDotx[i * (segments + 3) + j] -= pdGDotDotx[j];
            pdADotDoty[i * (segments + 3) + j] -= pdGDotDoty[j];
        }
    }

/*+-------------+
  |Fill f arrays|
  +-------------+*/

    for (i = 1; i <= segments; i++) {
        double c = cos(x->theta[i - 1]);
        double s = sin(x->theta[i - 1]);
        const int line = i * (segments + 3);
        const int prevLine = (i - 1) * (segments + 3);
        double F = -parameters[2] * parameters[0] * 0.5 * (-(pdADotx[i] + pdADotx[i - 1]) * s + (pdADoty[i] + pdADoty[i - 1]) * c);
        int j = segments + 3;

        for (; --j >= 0;) {
            pdfx[line + j] = pdfx[prevLine + j] + parameters[1] * 0.5 * (pdADotDotx[line + j] + pdADotDotx[prevLine + j]);
            pdfy[line + j] = pdfy[prevLine + j] + parameters[1] * 0.5 * (pdADotDoty[line + j] + pdADotDoty[prevLine + j]);
        }

        pdfx[line + segments + 2] += -F * s;
        pdfy[line + segments + 2] += F * c;
    }

/*+--------------------------------------+
  |Compute the linear system to be solved|
  +--------------------------------------+*/

    for (i = segments + 3; --i >= 0;) {
        pdMatrix[i] = pdfx[segments * (segments + 3) + i];
        pdMatrix[i + segments + 3] = pdfy[segments * (segments + 3) + i];
    }

    for (i = 1; i <= segments; i++) {
        double c = cos(x->theta[i - 1]);
        double s = sin(x->theta[i - 1]);
        double thetaDot = x->thetaDot[i - 1];
        int matrixLine = (i + 1) * (segments + 3);
        int line = i * (segments + 3);
        int prevLine = (i - 1) * (segments + 3);
        int j = segments + 3;

        for (; --j >= 0;)
            pdMatrix[matrixLine + j] = parameters[0] * 0.5 * (c * (pdfy[line + j] + pdfy[prevLine + j]) - s * (pdfx[line + j] + pdfx[prevLine + j]));

        pdMatrix[matrixLine + segments + 2] += parameters[2] * thetaDot * parameters[0] * parameters[0] * parameters[0] / 12;
        pdMatrix[matrixLine + i + 1] -= parameters[1] * parameters[0] / 12;
    }

/*+-----------------------+
  |LU-Decompose its matrix|
  +-----------------------+*/

    for (i = segments + 2; --i >= 0;) {
        int j = segments + 2;
        for (; --j >= 0;)
            pdLU[i * (segments + 2) + j] = pdMatrix[i * (segments + 3) + j];
        pdb[i] = pdMatrix[i * (segments + 3) + segments + 2];
    }

}


/* Returns a triplet containing the next state, the applied action and the reward given the current state and action. */

char nextStateReward(state* s, double* a, state** nextState,double* reward) {

    *nextState = copyState(s);

    if(s->isTerminal < 0) {
        *reward = 0.0;
    } else {
        double deltaT = timeStep / 8.0;
        unsigned int t = 0;
        for(; t < 8; t++) {
            int i = segments + 2;
        	int signum = 0;
            gsl_matrix_view MpdLU;
            gsl_permutation *perm = NULL;
            gsl_vector_view vpdRH;
            gsl_vector_view vpdSolution;

            computeMatrix(*nextState);

        	for (; --i >= 0;)
        		pdRH[i] = pdb[i];

        	for (i = 0; i < segments - 1; i++) {
        		pdRH[i + 3] -= ((parameters[3] + parameters[3]) * a[i]) - parameters[3];
	        	pdRH[i + 2] += ((parameters[3] + parameters[3]) * a[i]) - parameters[3];
	        }

            MpdLU = gsl_matrix_view_array(pdLU, segments + 2, segments + 2);
            perm = gsl_permutation_alloc(segments + 2);
            gsl_linalg_LU_decomp(&MpdLU.matrix, perm, &signum);
            vpdRH = gsl_vector_view_array(pdRH, segments + 2);
            vpdSolution = gsl_vector_view_array(pdSolution, segments + 2);
            gsl_linalg_LU_solve(&MpdLU.matrix, perm, &vpdRH.vector, &vpdSolution.vector);

            (*nextState)->G[0] += deltaT * (*nextState)->GDot[0];
            (*nextState)->G[1] += deltaT * (*nextState)->GDot[1];

            (*nextState)->GDot[0] += deltaT * pdSolution[0];
            (*nextState)->GDot[1] += deltaT * pdSolution[1];

            for(i = 0; i < segments; i++) {
                (*nextState)->theta[i] += deltaT * s->thetaDot[i];
                if((*nextState)->theta[i] > M_PIl)
                    (*nextState)->theta[i] -= 2 * M_PIl;
                if((*nextState)->theta[i] < -M_PIl)
                    (*nextState)->theta[i] += 2 * M_PIl;
                (*nextState)->thetaDot[i] += deltaT * pdSolution[i + 2];
            }

            (*nextState)->AZero[0] += deltaT * pdADotx[0];
            (*nextState)->AZero[1] += deltaT * pdADoty[0];

            gsl_permutation_free(perm);

        }

        *reward = 1.0 - sqrt(pow((*nextState)->G[0] - parameters[4],2) + pow((*nextState)->G[1] - parameters[5],2)) / (parameters[6] * sqrt(2.0));

        if(*reward < 0.0)
            *reward = 0.0;
        if(*reward > 1.0)
            *reward = 1.0;



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

    printf("firstPosition: (% f, % f) GPosition: (% f, % f) ",s->AZero[0], s->AZero[1], s->G[0], s->G[1]);

}


void printAction(double* a) {

    unsigned int i = 1;

    printf("action: (% f", ((parameters[3] + parameters[3]) * a[0]) - parameters[3]);

    for(; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++)
        printf(",% f", ((parameters[3] + parameters[3]) * a[i]) - parameters[3]);

    printf(") ");

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
