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
#include <string.h>
#define __USE_GNU
#include <math.h>
#undef __USE_GNU

#ifdef USE_SDL
    #include <SDL/SDL.h>
    #include <SDL/SDL_gfxPrimitives.h>
#endif

#include "lipschitzian.h"

#define INCREMENT_STEP_SUBSPACES_ARRAY 32

/*+------------------------------------------------------+
  | Initialize an instance of the lipschitzian algorithm |
  +------------------------------------------------------+*/

lipschitzian_instance* lipschitzian_initInstance(state* initial, double discountFactor, double L) {

    lipschitzian_instance* instance = (lipschitzian_instance*)malloc(sizeof(lipschitzian_instance));
    unsigned int i = 1;

    instance->gammaPowers[0] = 1.0;

    for(; i < NB_COMPUTED_POWER; i++)
        instance->gammaPowers[i] = instance->gammaPowers[i - 1] * discountFactor;

    instance->L = L;
    instance->gamma = discountFactor;

    instance->subsets = NULL;
    instance->list = NULL;

    if(initial != NULL)
        lipschitzian_resetInstance(instance, initial);

    return instance;

}


/*+--------------------------+
  | Free the tree of subsets |
  +--------------------------+*/

static void freeSubsets(lipschitzian_subset* subset) {

    while(subset != NULL) {
        unsigned int i = 0;
        lipschitzian_subset* next = subset->next;

        for(; i <= (subset->n+1); i++)
          freeState(subset->subspaces[i].s);

        free(subset->subspaces);
        free(subset);

        subset = next;
    }

}


/*+--------------------------------------------+
  | Reset an instance with a new initial state |
  +--------------------------------------------+*/

void lipschitzian_resetInstance(lipschitzian_instance* instance, state* initial) {

    unsigned int i = 0;

    if(instance->subsets != NULL)
        freeSubsets(instance->list);

    instance->subsets = (lipschitzian_subset*)malloc(sizeof(lipschitzian_subset));

    instance->list = instance->subsets;

    instance->subsets->subspaces = (lipschitzian_subspace*)malloc(sizeof(lipschitzian_subspace) * INCREMENT_STEP_SUBSPACES_ARRAY);

    instance->subsets->maxCrtNbSubspaces = INCREMENT_STEP_SUBSPACES_ARRAY;
    instance->subsets->n = 0;

    instance->subsets->maxBoundedChild = NULL;
    instance->subsets->father = NULL;
    instance->subsets->leftChild = NULL;
    instance->subsets->rightChild = NULL;

    instance->subsets->subspaces[0].s = copyState(initial);

    for(; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++) {
        instance->subsets->subspaces[0].action[i] = 0.5;
        instance->subsets->subspaces[0].halfSidesLength[i] = 0.5;
    }

    instance->subsets->subspaces[0].delta = 0.5*sqrt(NUMBER_OF_DIMENSIONS_OF_ACTION);

    instance->subsets->subspaces[0].nextCutDimension = 0;
    instance->subsets->subspaces[0].nextDelta = sqrt((0.25 * (NUMBER_OF_DIMENSIONS_OF_ACTION - 1)) + (1.0 / 36.0));

    instance->subsets->subspaces[1].isClosedPath = nextStateReward(initial, instance->subsets->subspaces[0].action, &(instance->subsets->subspaces[1].s), &(instance->subsets->subspaces[0].reward)) < 0 ? 1 : 0;

    instance->subsets->subspaces[0].discountedSumOfRewards = instance->subsets->subspaces[0].reward;

    instance->subsets->bound = (instance->subsets->subspaces[0].reward + (instance->L * instance->subsets->subspaces[0].delta) < 1.0 ? instance->subsets->subspaces[0].reward + (instance->L * instance->subsets->subspaces[0].delta) : 1.0) + (instance->gamma / (1.0 - instance->gamma));

    instance->nextSubsetToDiscretize = instance->subsets;
    instance->nextNodeToAppendTo = instance->subsets;

    instance->maxDiscountedSumOfRewards = instance->subsets->subspaces[0].reward;
    memcpy(instance->crtOptimalAction, instance->subsets->subspaces[0].action, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    instance->crtOptimalSequence = instance->subsets;

    instance->maxDepth = 1;	

    instance->crtNbSubspaces = 1;
    instance->crtNbSubsets = 1;

    instance->subsets->constrainedUntil = 0;

    instance->list->next = NULL;

    instance->subsets->subspaces[0].headsSameState = instance->subsets;
    instance->subsets->subspaces[0].nextsSameState = NULL;

}


/*+-----------------------------------------------------+
  | Propagate the new bound through the tree of subsets |
  +-----------------------------------------------------+*/

static void propagateBound(lipschitzian_instance* instance, lipschitzian_subset* startingPoint) {

    lipschitzian_subset* crt = startingPoint;
    while(crt != NULL) {
        if(crt->rightChild != NULL) {
            crt->maxBoundedChild = crt->rightChild;

            if(crt->rightChild->maxBoundedChild != NULL) {
                if(crt->rightChild->maxBoundedChild->bound > crt->maxBoundedChild->bound)
                    crt->maxBoundedChild = crt->rightChild->maxBoundedChild;

                if(crt->leftChild->maxBoundedChild->bound > crt->maxBoundedChild->bound)
                    crt->maxBoundedChild = crt->leftChild->maxBoundedChild;
            } else {
                if((crt->leftChild->maxBoundedChild != NULL) && (crt->leftChild->maxBoundedChild->bound > crt->maxBoundedChild->bound))
                    crt->maxBoundedChild = crt->leftChild->maxBoundedChild;
            }

            if(crt->leftChild->bound > crt->maxBoundedChild->bound)
                crt->maxBoundedChild = crt->leftChild;
        }

        crt = crt->father;
    }

    if((instance->subsets->maxBoundedChild != NULL) && (instance->subsets->maxBoundedChild->bound > instance->subsets->bound))
        instance->nextSubsetToDiscretize = instance->subsets->maxBoundedChild;
    else
        instance->nextSubsetToDiscretize = instance->subsets;

}


/*+--------------------+
  | Trisect a subspace |
  +--------------------+*/

static void trisectSubspace(lipschitzian_instance* instance, unsigned int min, double minPartialSumDelta, double minPartialNewBound, unsigned int* crtNbEvaluations) {

    unsigned int i = 0;

    double tentativelySumDelta = 0.0;
    double tentativelyNewBound = 0.0;

    lipschitzian_subset* crtNode = instance->nextNodeToAppendTo->father;
    lipschitzian_subset* prevNode = instance->nextNodeToAppendTo;

    lipschitzian_subset* discretizedSubset = instance->nextSubsetToDiscretize;

    lipschitzian_subset* leftSubset = (lipschitzian_subset*)malloc(sizeof(lipschitzian_subset));
    lipschitzian_subset* rightSubset = (lipschitzian_subset*)malloc(sizeof(lipschitzian_subset));

    double shift = (discretizedSubset->subspaces[min].halfSidesLength[discretizedSubset->subspaces[min].nextCutDimension] * 2.0) / 3.0;
    unsigned int cutDimension = discretizedSubset->subspaces[min].nextCutDimension;

    leftSubset->constrainedUntil = 0;
    rightSubset->constrainedUntil = 0;

    if(min < discretizedSubset->n) {
        leftSubset->constrainedUntil = discretizedSubset->n;
        rightSubset->constrainedUntil = discretizedSubset->n;
    }

    if(discretizedSubset->n < discretizedSubset->constrainedUntil) {
        leftSubset->constrainedUntil = discretizedSubset->constrainedUntil;
        rightSubset->constrainedUntil = discretizedSubset->constrainedUntil;
    }

    leftSubset->subspaces = (lipschitzian_subspace*)malloc(sizeof(lipschitzian_subspace) * discretizedSubset->maxCrtNbSubspaces);
    rightSubset->subspaces = (lipschitzian_subspace*)malloc(sizeof(lipschitzian_subspace) * discretizedSubset->maxCrtNbSubspaces);

    leftSubset->maxCrtNbSubspaces = discretizedSubset->maxCrtNbSubspaces;
    rightSubset->maxCrtNbSubspaces = discretizedSubset->maxCrtNbSubspaces;

    discretizedSubset->subspaces[min].delta = discretizedSubset->subspaces[min].nextDelta;
    discretizedSubset->subspaces[min].halfSidesLength[cutDimension] /= 3.0;

    if(NUMBER_OF_DIMENSIONS_OF_ACTION == 1) {
        discretizedSubset->subspaces[min].nextDelta = discretizedSubset->subspaces[min].halfSidesLength[0] / 3.0;
    } else {
        discretizedSubset->subspaces[min].nextCutDimension++;
    
        if(discretizedSubset->subspaces[min].nextCutDimension == NUMBER_OF_DIMENSIONS_OF_ACTION)
            discretizedSubset->subspaces[min].nextCutDimension = 0;

        if(discretizedSubset->subspaces[min].nextCutDimension == (NUMBER_OF_DIMENSIONS_OF_ACTION - 1)) {
            discretizedSubset->subspaces[min].nextDelta = discretizedSubset->subspaces[min].halfSidesLength[0] * sqrt(NUMBER_OF_DIMENSIONS_OF_ACTION);
        } else {
            discretizedSubset->subspaces[min].nextDelta = 0.0;

            for(; i < discretizedSubset->subspaces[min].nextCutDimension; i++)
                discretizedSubset->subspaces[min].nextDelta += (discretizedSubset->subspaces[min].halfSidesLength[i] * discretizedSubset->subspaces[min].halfSidesLength[i]);

            discretizedSubset->subspaces[min].nextDelta += ((discretizedSubset->subspaces[min].halfSidesLength[i] * discretizedSubset->subspaces[min].halfSidesLength[i]) / 9.0);

            for(i+=1; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++)
                discretizedSubset->subspaces[min].nextDelta += (discretizedSubset->subspaces[min].halfSidesLength[i] * discretizedSubset->subspaces[min].halfSidesLength[i]);

            discretizedSubset->subspaces[min].nextDelta = sqrt(discretizedSubset->subspaces[min].nextDelta);
        }
    }

    if(discretizedSubset->n < leftSubset->constrainedUntil) {
        memcpy(leftSubset->subspaces, discretizedSubset->subspaces, sizeof(lipschitzian_subspace) * (leftSubset->constrainedUntil + 1));
        memcpy(rightSubset->subspaces, discretizedSubset->subspaces, sizeof(lipschitzian_subspace) * (leftSubset->constrainedUntil + 1));
    } else {
        memcpy(leftSubset->subspaces, discretizedSubset->subspaces, sizeof(lipschitzian_subspace) * (discretizedSubset->n + 1));
        memcpy(rightSubset->subspaces, discretizedSubset->subspaces, sizeof(lipschitzian_subspace) * (discretizedSubset->n + 1));
    }

    leftSubset->subspaces[min].action[cutDimension] -= shift;
    rightSubset->subspaces[min].action[cutDimension] += shift;


    for(i = 0; i <= min; i++) {
        leftSubset->subspaces[i].s = copyState(discretizedSubset->subspaces[i].s);
        rightSubset->subspaces[i].s = copyState(discretizedSubset->subspaces[i].s);
    }

    leftSubset->subspaces[min + 1].isClosedPath = nextStateReward(leftSubset->subspaces[min].s, leftSubset->subspaces[min].action, &(leftSubset->subspaces[min + 1].s), &(leftSubset->subspaces[min].reward)) < 0 ? 1 : 0;
    rightSubset->subspaces[min + 1].isClosedPath = nextStateReward(rightSubset->subspaces[min].s, rightSubset->subspaces[min].action, &(rightSubset->subspaces[min + 1].s), &(rightSubset->subspaces[min].reward)) < 0 ? 1 : 0;
    (*crtNbEvaluations) += 2;

    if(min == 0) {
        leftSubset->subspaces[0].discountedSumOfRewards = leftSubset->subspaces[0].reward;
        rightSubset->subspaces[0].discountedSumOfRewards = rightSubset->subspaces[0].reward;
    } else {
        leftSubset->subspaces[min].discountedSumOfRewards = leftSubset->subspaces[min - 1].discountedSumOfRewards + (instance->gammaPowers[min] * leftSubset->subspaces[min].reward);
        rightSubset->subspaces[min].discountedSumOfRewards = rightSubset->subspaces[min - 1].discountedSumOfRewards + (instance->gammaPowers[min] * rightSubset->subspaces[min].reward);
    }

    if(leftSubset->subspaces[min].discountedSumOfRewards > instance->maxDiscountedSumOfRewards) {
        instance->maxDiscountedSumOfRewards = leftSubset->subspaces[min].discountedSumOfRewards;
        memcpy(instance->crtOptimalAction, leftSubset->subspaces[0].action, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
        instance->crtOptimalSequence = leftSubset;
    }
    if(rightSubset->subspaces[min].discountedSumOfRewards > instance->maxDiscountedSumOfRewards) {
        instance->maxDiscountedSumOfRewards = rightSubset->subspaces[min].discountedSumOfRewards;
        memcpy(instance->crtOptimalAction, rightSubset->subspaces[0].action, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
        instance->crtOptimalSequence = rightSubset;
    }

    leftSubset->n = min;
    rightSubset->n = min;

    tentativelySumDelta = (minPartialSumDelta + discretizedSubset->subspaces[min].delta) * instance->L;


    if((tentativelySumDelta + leftSubset->subspaces[min].reward) > 1.0)
        leftSubset->bound = minPartialNewBound + (instance->gammaPowers[min] / (1.0 - instance->gamma));
    else
        leftSubset->bound = minPartialNewBound + (instance->gammaPowers[min] * (tentativelySumDelta + leftSubset->subspaces[min].reward)) + (instance->gammaPowers[min + 1] / (1.0 - instance->gamma));

    if((tentativelySumDelta + rightSubset->subspaces[min].reward) > 1.0)
        rightSubset->bound = minPartialNewBound + (instance->gammaPowers[min] / (1.0 - instance->gamma));
    else
        rightSubset->bound = minPartialNewBound + (instance->gammaPowers[min] * (tentativelySumDelta + rightSubset->subspaces[min].reward)) + (instance->gammaPowers[min + 1] / (1.0 - instance->gamma));


/*************************** NOT SURE AT ALL IT'S A GOOD IDEA ****************************************************/

    /*if((tentativelySumDelta + leftSubset->subspaces[min].reward) < 1.0) {
        tentativelyNewBound = minPartialNewBound + (instance->gammaPowers[min] * (tentativelySumDelta + leftSubset->subspaces[min].reward));
        
        for(i = min + 1; i < leftSubset->constrainedUntil; i++) {
            tentativelySumDelta = (tentativelySumDelta + (leftSubset->subspaces[i].delta * 2.0)) * instance->L;

            if(tentativelySumDelta > 1.0)
                break;

            tentativelyNewBound += (instance->gammaPowers[i] * tentativelySumDelta);
        }

        tentativelyNewBound += (instance->gammaPowers[i] / (1.0 - instance->gamma));
    } else
        tentativelyNewBound = minPartialNewBound + (instance->gammaPowers[min] / (1.0 - instance->gamma));

    leftSubset->bound = tentativelyNewBound;

    if((tentativelySumDelta + rightSubset->subspaces[min].reward) < 1.0) {
        tentativelyNewBound = minPartialNewBound + (instance->gammaPowers[min] * (tentativelySumDelta + rightSubset->subspaces[min].reward));

        for(i = min + 1; i < rightSubset->constrainedUntil; i++) {
            tentativelySumDelta = (tentativelySumDelta + (rightSubset->subspaces[i].delta * 2.0)) * instance->L;
    
            if(tentativelySumDelta > 1.0)
                break;

            tentativelyNewBound += (instance->gammaPowers[i] * tentativelySumDelta);
        }

        tentativelyNewBound += (instance->gammaPowers[i] / (1.0 - instance->gamma));
    } else
        tentativelyNewBound = minPartialNewBound + (instance->gammaPowers[min] / (1.0 - instance->gamma));

    rightSubset->bound = tentativelyNewBound;*/

/*****************************************************************************************************************/

    tentativelyNewBound = minPartialNewBound + (instance->gammaPowers[min] * (tentativelySumDelta + discretizedSubset->subspaces[min].reward));
    for(i = min + 1; i <= discretizedSubset->n; i++) {
        tentativelySumDelta = (tentativelySumDelta + discretizedSubset->subspaces[i].delta) * instance->L;
        if((tentativelySumDelta + discretizedSubset->subspaces[i].reward) > 1.0)
            break;
        tentativelyNewBound += (instance->gammaPowers[i] * (tentativelySumDelta + discretizedSubset->subspaces[i].reward));
    }

/*************************** NOT SURE AT ALL IT'S A GOOD IDEA ****************************************************/

    /*if(i > discretizedSubset->n) {
        for(; i <= discretizedSubset->constrainedUntil; i++) {
        tentativelySumDelta = (tentativelySumDelta + (discretizedSubset->subspaces[i].delta * 2.0)) * instance->L;
        if(tentativelySumDelta > 1.0)
            break;
            tentativelyNewBound += (instance->gammaPowers[i] * tentativelySumDelta);
        }
    }*/

/*****************************************************************************************************************/

    discretizedSubset->bound = tentativelyNewBound + (instance->gammaPowers[i] / (1.0 - instance->gamma));

    instance->nextNodeToAppendTo->leftChild = leftSubset;
    instance->nextNodeToAppendTo->rightChild = rightSubset;

    leftSubset->maxBoundedChild = NULL;
    rightSubset->maxBoundedChild = NULL;

    leftSubset->leftChild = NULL;
    leftSubset->rightChild = NULL;
    rightSubset->leftChild = NULL;
    rightSubset->rightChild = NULL;

    leftSubset->father = instance->nextNodeToAppendTo;
    rightSubset->father = instance->nextNodeToAppendTo;

    propagateBound(instance, instance->nextNodeToAppendTo);
    propagateBound(instance, discretizedSubset);

    while((crtNode != NULL) && (crtNode->rightChild == prevNode)) {
        prevNode = crtNode;
        crtNode = crtNode->father;
    }

    if(crtNode == NULL)
        crtNode = instance->subsets;
    else
        crtNode = crtNode->rightChild;

    while(crtNode->leftChild != NULL)
        crtNode = crtNode->leftChild;

    instance->nextNodeToAppendTo = crtNode;		

    instance->crtNbSubspaces += ((min + 1) * 2);
    instance->crtNbSubsets += 2;

    leftSubset->next = instance->list;
    rightSubset->next = leftSubset;
    instance->list = rightSubset;

    rightSubset->subspaces[min].nextsSameState = leftSubset;
    leftSubset->subspaces[min].nextsSameState = discretizedSubset->subspaces[min].headsSameState->subspaces[min].nextsSameState;
    discretizedSubset->subspaces[min].headsSameState->subspaces[min].nextsSameState = rightSubset;

}


/*+---------------------------------------------------------------+
  | Computed the next Lipschitz coefficient based on the instance |
  +---------------------------------------------------------------+*/

double lipschitzian_computeNextL(lipschitzian_instance* instance) {

/*
    double L = 0.0;
    lipschitzian_subset* crt1 = instance->list;

    while(crt1 != NULL) {
        lipschitzian_subset* crt2 = crt1->next;

        while(crt2 != NULL) {
            if(crt1->subspaces[0].action[0] != crt2->subspaces[0].action[0]) {
                double computedL = fabs(crt1->subspaces[0].reward - crt2->subspaces[0].reward) / fabs(crt1->subspaces[0].action[0] - crt2->subspaces[0].action[0]);

                if(computedL > L)
                    L = computedL;
            }

            crt2 = crt2->next;
        }

        crt1 = crt1->next;
    }

    return L;
*/

    double L = 0.0;
    unsigned int depth = 0;

    for(; depth <= instance->crtOptimalSequence->n; depth++) {
        lipschitzian_subset* crt = instance->crtOptimalSequence->subspaces[depth].headsSameState;

        while(crt != NULL) {
            lipschitzian_subset* other = crt->subspaces[depth].nextsSameState;

            while(other != NULL) {
                if(crt->subspaces[depth].action[0] != other->subspaces[depth].action[0]) {
                    double computedL = fabs(crt->subspaces[depth].reward - other->subspaces[depth].reward) / fabs(crt->subspaces[depth].action[0] - other->subspaces[depth].action[0]);
                    if(computedL > L)
                        L = computedL;
                }

                other = other->subspaces[depth].nextsSameState;
            }

            crt = crt->subspaces[depth].nextsSameState;
        }
    }

    return L;

}


/*+------------------------------------------------------------------------+
  | Launch the lipschitzian algorithm with a limited number of evaluations |
  +------------------------------------------------------------------------+*/

double* lipschitzian_planning(lipschitzian_instance* instance, unsigned int maxNbEvaluations) {

    unsigned int crtNbEvaluations = 0;
    double* optimalAction = (double*)malloc(sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);

    while(crtNbEvaluations <= maxNbEvaluations) {
        lipschitzian_subset* discretizedSubset = instance->nextSubsetToDiscretize;

        unsigned int T = 0;
        double partialSumDelta = 0.0;
        double partialNewBound = 0.0;
        double minNewBound = 1.0 / (1.0 - instance->gamma);
        unsigned int min = 0;
        double minPartialNewBound = 0.0;
        double minPartialSumDelta = 0.0;
        unsigned int i = 0;

        for(; T <= discretizedSubset->n; T++) {
            partialSumDelta = (partialSumDelta + discretizedSubset->subspaces[T].delta) * instance->L;
            
            if(discretizedSubset->subspaces[T].reward + partialSumDelta > 1.0)
              break;
        }

        if(T != 0)
            T--;

        partialSumDelta = 0.0;

        for(; i <= T; i++) {
            unsigned int j = 0;
            double tentativelySumDelta = (partialSumDelta + discretizedSubset->subspaces[i].nextDelta) * instance->L;
            double tentativelyNewBound = partialNewBound + (instance->gammaPowers[i] * (discretizedSubset->subspaces[i].reward + tentativelySumDelta));

            for(j = i + 1; j <= discretizedSubset->n; j++) {
                tentativelySumDelta = (tentativelySumDelta + discretizedSubset->subspaces[j].delta) * instance->L;

                if((discretizedSubset->subspaces[j].reward + tentativelySumDelta) > 1.0)
                    break;

                tentativelyNewBound += (instance->gammaPowers[j] * (discretizedSubset->subspaces[j].reward + tentativelySumDelta));
            }

/*************************** NOT SURE AT ALL IT'S A GOOD IDEA ****************************************************/

            /*if(j > discretizedSubset->n) {
            for(; j <= discretizedSubset->constrainedUntil; j++) {
                tentativelySumDelta = (tentativelySumDelta + (discretizedSubset->subspaces[j].delta * 2.0)) * instance->L;
                if(tentativelySumDelta > 1.0)
                    break;
                tentativelyNewBound += (instance->gammaPower[j] * tentativelySumDelta);
            }
            }*/

/*****************************************************************************************************************/

            tentativelyNewBound += (instance->gammaPowers[j] / (1.0 - instance->gamma));

            if(tentativelyNewBound < minNewBound) {
                minNewBound = tentativelyNewBound;
                minPartialSumDelta = partialSumDelta;
                minPartialNewBound = partialNewBound;
                min = i;
            }

            partialSumDelta = (partialSumDelta + discretizedSubset->subspaces[i].delta) * instance->L;
            partialNewBound += (instance->gammaPowers[i] * (discretizedSubset->subspaces[i].reward + partialSumDelta));

        }

        if((T == discretizedSubset->n) && !discretizedSubset->subspaces[discretizedSubset->n + 1].isClosedPath) {
            double tentativelySumDelta = (partialSumDelta + (discretizedSubset->constrainedUntil > T ? (discretizedSubset->subspaces[T + 1].delta * 2.0) : sqrt(NUMBER_OF_DIMENSIONS_OF_ACTION) )) * instance->L;
            double tentativelyNewBound = partialNewBound + (tentativelySumDelta > 1.0 ? instance->gammaPowers[T + 1] / (1.0 - instance->gamma) : (instance->gammaPowers[T + 1] * tentativelySumDelta) + (instance->gammaPowers[T + 2] / (1.0 - instance->gamma)));

/*************************** NOT SURE AT ALL IT'S A GOOD IDEA ****************************************************/

            /*double tentativelyNewBound = partialNewBound + (instance->gammaPowers[T + 1] * (tentativelySumDelta > 1.0 ? 1.0 : tentativelySumDelta));

            if((tentativelySumDelta < 1.0) && (discretizedSubset->constrainedUntil > (T + 1) )) {
            for(i = T + 2; i < discretizedSubset->constrainedUntil; i++) {
            tentativelySumDelta = (tentativelySumDelta + (discretizedSubset->subspaces[i].delta * 2.0)) * instance->L;
            if(tentativelySumDelta > 1.0)
            break;
            tentativelyNewBound += (instance->gammaPowers[i] * tentativelySumDelta);
            }
            tentativelyNewBound += (instance->gammaPowers[i] / (1.0 - instance->gamma));
            } else {
            tentativelyNewBound += (instance->gammaPowers[T + 2] / (1.0 - instance->gamma));
            }*/

/*****************************************************************************************************************/

            if(tentativelyNewBound < minNewBound) {		/* A new subspace will be added to the discretized subset */

                double tmp = -1;

                discretizedSubset->n++;

                if((discretizedSubset->n + 1) == discretizedSubset->maxCrtNbSubspaces) {
                    discretizedSubset->maxCrtNbSubspaces += INCREMENT_STEP_SUBSPACES_ARRAY;
                    discretizedSubset->subspaces = (lipschitzian_subspace*)realloc(discretizedSubset->subspaces, sizeof(lipschitzian_subspace) * discretizedSubset->maxCrtNbSubspaces);
                }

                if(discretizedSubset->constrainedUntil <= T) {
                    for(i = 0; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++) {
                        discretizedSubset->subspaces[discretizedSubset->n].action[i] = 0.5;
                        discretizedSubset->subspaces[discretizedSubset->n].halfSidesLength[i] = 0.5;
                    }

                    discretizedSubset->subspaces[discretizedSubset->n].delta = 0.5 * sqrt(NUMBER_OF_DIMENSIONS_OF_ACTION);
                    discretizedSubset->subspaces[discretizedSubset->n].nextCutDimension = 0;
                    discretizedSubset->subspaces[discretizedSubset->n].nextDelta = sqrt((0.25 * (NUMBER_OF_DIMENSIONS_OF_ACTION - 1)) + (1.0 / 36.0));

                }

                tmp = discretizedSubset->subspaces[discretizedSubset->n].action[0];


                discretizedSubset->subspaces[discretizedSubset->n + 1].isClosedPath = nextStateReward(discretizedSubset->subspaces[discretizedSubset->n].s, discretizedSubset->subspaces[discretizedSubset->n].action, &(discretizedSubset->subspaces[discretizedSubset->n + 1].s), &(discretizedSubset->subspaces[discretizedSubset->n].reward)) < 0 ? 1 : 0;
                crtNbEvaluations++;

                discretizedSubset->subspaces[discretizedSubset->n].discountedSumOfRewards = discretizedSubset->subspaces[discretizedSubset->n - 1].discountedSumOfRewards + (instance->gammaPowers[discretizedSubset->n] * discretizedSubset->subspaces[discretizedSubset->n].reward);

                if(discretizedSubset->subspaces[discretizedSubset->n].discountedSumOfRewards > instance->maxDiscountedSumOfRewards) {
                    instance->maxDiscountedSumOfRewards = discretizedSubset->subspaces[discretizedSubset->n].discountedSumOfRewards;
                    memcpy(instance->crtOptimalAction, discretizedSubset->subspaces[0].action, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
                    instance->crtOptimalSequence = discretizedSubset;
                }

                tentativelySumDelta = (partialSumDelta + discretizedSubset->subspaces[discretizedSubset->n].delta) * instance->L;

                if(tentativelySumDelta + discretizedSubset->subspaces[discretizedSubset->n].reward > 1.0)
                    discretizedSubset->bound = partialNewBound + (instance->gammaPowers[discretizedSubset->n] / (1.0 - instance->gamma));
                else				
                    discretizedSubset->bound = partialNewBound + (instance->gammaPowers[discretizedSubset->n] * (tentativelySumDelta + discretizedSubset->subspaces[discretizedSubset->n].reward)) + (instance->gammaPowers[discretizedSubset->n + 1] / (1.0 - instance->gamma));

/*************************** NOT SURE AT ALL IT'S A GOOD IDEA ****************************************************/

                /*if(tentativelySumDelta + discretizedSubset->subspaces[discretizedSubset->n].reward < 1.0) {
                    tentativelyNewBound = partialNewBound + (instance->gammaPowers[discretizedSubset->n] * (tentativelySumDelta + discretizedSubset->subspaces[discretizedSubset->n].reward));

                    for(i = T + 2; i < discretizedSubset->constrainedUntil; i++) {
                        tentativelySumDelta = (tentativelySumDelta + (discretizedSubset->subspaces[i].delta * 2.0)) * instance->L;

                        if(tentativelySumDelta > 1.0)
                            break;

                        tentativelyNewBound += (instance->gammaPowers[i] * tentativelySumDelta);
                    }
                    
                    tentativelyNewBound += (instance->gammaPowers[i] / (1.0 - instance->gamma));
                } else {
                    tentativelyNewBound = partialNewBound + (instance->gammaPowers[discretizedSubset->n] / (1.0 - instance->gamma));
                }

                discretizedSubset->bound = tentativelyNewBound;*/

/*****************************************************************************************************************/

                propagateBound(instance, discretizedSubset);

                if((discretizedSubset->n + 1) > instance->maxDepth)
                    instance->maxDepth = discretizedSubset->n + 1;

                instance->crtNbSubspaces++;

                discretizedSubset->subspaces[discretizedSubset->n].headsSameState = discretizedSubset;
                discretizedSubset->subspaces[discretizedSubset->n].nextsSameState = NULL;
            } else {						/* The min-th subspace will be trisected and 2 new subsets will be created */
                trisectSubspace(instance, min, minPartialSumDelta, minPartialNewBound, &crtNbEvaluations);
            }
        } else {							/* The min-th subspace will be trisected and 2 new subsets will be created */
            trisectSubspace(instance, min, minPartialSumDelta, minPartialNewBound, &crtNbEvaluations);
        }
    }

    return memcpy(optimalAction, instance->crtOptimalAction, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);

}


/*+----------------------------------------+
  | Compute the mean depth of the instance |
  +----------------------------------------+*/

double lipschitzian_getMeanDepth(lipschitzian_instance* instance) {

    lipschitzian_subset* crt = instance->list;
    double meanDepth = 0;
    
    while(crt != 0) {
        meanDepth += crt->n;
        crt = crt->next;
    }

    return meanDepth / (double)instance->crtNbSubsets;
    
}


/*+--------------------------------------------------+
  | Uninit an instance of the lipschitzian algorithm |
  +--------------------------------------------------+*/

void lipschitzian_uninitInstance(lipschitzian_instance** instance) {

    if((*instance)->subsets != NULL)
        freeSubsets((*instance)->list);

    free(*instance);
    *instance = NULL;

}


#ifdef USE_SDL
/*+----------------------------------------+
  | Draw the subspace onto the SDL_Surface |
  +----------------------------------------+*/
#if NUMBER_OF_DIMENSIONS_OF_ACTION == 1
static void drawSubspaces(SDL_Surface* screen, int halfScreenWidth, lipschitzian_subset* node, unsigned int hSpace) {

	unsigned int i = 0;
	
	filledCircleRGBA(screen, halfScreenWidth * (1 + node->subspaces[0].action[0]), 10, 2, 0, 0, 0, 255);
	
	for(; i < node->n; i++) {
		aalineRGBA(screen, halfScreenWidth * (1 + node->subspaces[i].action[0]), 10 + (i * hSpace), halfScreenWidth * (1 + node->subspaces[i+1].action[0]), 10 + ((i + 1) * hSpace), 0, 0, 0, 255);
		filledCircleRGBA(screen, halfScreenWidth * (1 + node->subspaces[i+1].action[0]), 10 + ((i + 1) * hSpace), 2, 0, 0, 0, 255);
	}

	if(node->leftChild) {
		drawSubspaces(screen, halfScreenWidth, node->leftChild, hSpace);
		drawSubspaces(screen, halfScreenWidth, node->rightChild, hSpace);
	}

}
#endif

/*+--------------------------------------------------------------------+
  | The fonction passed to the viewer for the displaying the algorithm |
  +--------------------------------------------------------------------+*/

void lipschitzian_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance) {

    #if NUMBER_OF_DIMENSIONS_OF_ACTION == 1
    drawSubspaces(screen, screenWidth / 2.0, ((lipschitzian_instance*)instance)->subsets, (screenHeight - 10) / ((lipschitzian_instance*)instance)->maxDepth);
    #else
    (void)screen;
    (void)screenWidth;
    (void)screenHeight;
    (void)instance;
    #endif

}
#endif
