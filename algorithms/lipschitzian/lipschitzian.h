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

#ifndef LIPSCHITZIAN_H
#define LIPSCHITZIAN_H

#ifdef USE_SDL
    #include <SDL/SDL.h>
#endif

#include "../../problems/generative_model.h"


/*+--------------------------------------+
  | Represents an action in the sequence |
  | as a subspace of the space defining  |
  | the infinity of sequences.           |
  +--------------------------------------+*/

typedef struct {

    double action[NUMBER_OF_DIMENSIONS_OF_ACTION];          /* The action of this subspace which is its center */
    double delta;                                           /* The half length of the diagonal of the subspace */

    /* Dealing with the intrisic continuous action space dimensions */

    double halfSidesLength[NUMBER_OF_DIMENSIONS_OF_ACTION]; /* The half length of each sides of the subspace */
    unsigned int nextCutDimension;                          /* The next dimension to be cut in the subspace in a round-robin fashion */
    double nextDelta;                                       /* The next half lenght of the diagonal if the subspace is trisected along one of its dimension according to nextCutDimension*/

    /*unsigned char isConstrained;*/

    state* s;                                               /* The state in which we will apply the action defined with the center of this subspace */
    double discountedSumOfRewards;                          /* The discounted sum of rewards gotten since the initial state */
    double reward;                                          /* The reward obtained by applying the action on the state. */
    char isClosedPath;

    /* Pointers used for estimating the next L */
    struct lipschitzian_subset* nextsSameState;
    struct lipschitzian_subset* headsSameState;

} lipschitzian_subspace;


/*+-------------------------------------+
  | Represents a sequence as a space of |
  | infinite dimension which possesses	|
  | a bound and a depth.				|
  +-------------------------------------+*/

typedef struct lipschitzian_subset {

    lipschitzian_subspace* subspaces;                              /* Keeps track of the subspaces properties for each of the action along the sequence */
    unsigned int maxCrtNbSubspaces;                         /* Contains the number of lipschitzian_subspace item allocated */

    unsigned int n;                                         /* The current cardinality of this subset */
    double bound;                                           /* THE bound of this subset */
    unsigned int constrainedUntil;                          /* The index of the subspaces until which they are constrained by a previous trisecting, 0 being none. */


    /* Tree sorting of the bounded subsets */
    struct lipschitzian_subset* maxBoundedChild;                   /* The maximum bounded child under this one */
    struct lipschitzian_subset* father;                            /* Link to the father of this node. NULL if this node is the root */
    struct lipschitzian_subset* leftChild;                         /* Left child */
    struct lipschitzian_subset* rightChild;                        /* Right child */

    /* Pointer to the next subset in the instance list */
    struct lipschitzian_subset* next;

} lipschitzian_subset;


/*+-------------------------------------+
  | Represents an instance of the       |
  | lipschitzian planning algorithm.    |
  +-------------------------------------+*/

#define NB_COMPUTED_POWER 32768

typedef struct {

    double L;                                               /* The lipschitz constant of this instance */
    double gamma;                                           /* The discount factor of this instance */

    double gammaPowers[NB_COMPUTED_POWER];                  /* Array with power of the discount factor */

    lipschitzian_subset* subsets;                                  /* Binary tree of the subsets for finding the maximum bounded subset */

    /* List of subsets */
    lipschitzian_subset* list;                                     /* Linked list of the subsets */

    lipschitzian_subset* nextSubsetToDiscretize;                   /* The next subset that will be discretized (typicaly the maximum bounded one) */
    lipschitzian_subset* nextNodeToAppendTo;                       /* Where in the binary we are going to put it */

    unsigned int maxDepth;                                  /* The maximum length of an encountered action sequence */

    double maxDiscountedSumOfRewards;                       /* The current maximum for the discounted sum of rewards */
    double crtOptimalAction[NUMBER_OF_DIMENSIONS_OF_ACTION];/* The current optimal continuous action */
    lipschitzian_subset* crtOptimalSequence;                       /* The current optimal sequence of action (thus a subset) */

    unsigned int crtNbSubspaces;                            /* Statistic about the number of subspaces created */
    unsigned int crtNbSubsets;                              /* Statistic about the number of subsets created */

} lipschitzian_instance;


lipschitzian_instance* lipschitzian_initInstance(state* initial, double discountFactor, double L);
void lipschitzian_resetInstance(lipschitzian_instance* instance, state* initial);
double lipschitzian_computeNextL(lipschitzian_instance* instance);
double* lipschitzian_planning(lipschitzian_instance* instance, unsigned int maxNbEvaluations);
double lipschitzian_getMeanDepth();
void lipschitzian_uninitInstance(lipschitzian_instance** instance);

#ifdef USE_SDL
void lipschitzian_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance);
#endif

#endif
