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

#ifndef GENERATIVE_MODEL_H
#define GENERATIVE_MODEL_H

/* Represent a state of the model */
typedef struct state state;

/*extern unsigned int actionDimensionality;*/           /* The number of dimension making up the action */
extern double timeStep;                             /* Time step between two state */

extern double* parameters;							/* Model's parameters */
extern unsigned int nbParameters;					/* Number of model's parameters */

/* Initialisation of the parameters. To call before anything else.*/
void initGenerativeModelParameters();

/* Initialisation of the generative model. To call after parameters initialisation. */
void initGenerativeModel();

/* Free the generative model. To call if the generative model has to be discarded. */
void freeGenerativeModel();

/* Free the generative model parameters. To call afer everything is finished. */
void freeGenerativeModelParameters();

/* Returns an allocated initial state of the model. */
state* initState();

/* Returns an allocated state initialized from the parsed string */
state* makeState(const char* str);

/* Returns the state and the reward given the current state and action. */
char nextStateReward(state* s, double* a, state** nextState, double* reward);

/* Return an allocated copy of the state s */
state* copyState(state* s);

void printState(state* s);

void printAction(double* a);

/* Free the state s */
void freeState(state* s);

#endif
