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
#include <math.h>
#include <time.h>
#include <string.h>

#include <gsl/gsl_rng.h>

#ifdef USE_SDL
    #include <SDL/SDL.h>
    #include <SDL/SDL_gfxPrimitives.h>
#endif

#include "random_search.h"
#include "../../problems/generative_model.h"

unsigned int h_max_default(random_search_instance* instance) {

    return (unsigned int)(log(instance->crtNbEvaluations) / log(1.0/instance->gamma));

}

unsigned int (*h_max)(random_search_instance*) = h_max_default;

random_search_instance* random_search_initInstance(state* initial, double discountFactor) {

    unsigned int i = 1;
    random_search_instance* instance = (random_search_instance*)malloc(sizeof(random_search_instance));

    instance->rng = NULL;
    instance->initial = NULL;

    instance->gamma = discountFactor;
    instance->gammaPowers[0] = 1.0;
    for(;i < RANDOM_SEARCH_MAX_DEPTH; i++)
        instance->gammaPowers[i] = instance->gammaPowers[i - 1] * discountFactor;

    if(initial != NULL)
        random_search_resetInstance(instance, initial);

    return instance;

}


void random_search_resetInstance(random_search_instance* instance, state* initial) {

    unsigned int i = 0;
    if(instance->initial != NULL)
            freeState(instance->initial);
    if(instance->rng == NULL)
        instance->rng = gsl_rng_alloc(gsl_rng_mt19937);

    gsl_rng_set(instance->rng, time(NULL));

    instance->initial = copyState(initial);

    instance->crtDepthLimit = 1;
    instance->crtNbEvaluations = 0;
    instance->crtMaxDepth = 0;
    instance->crtOptimalValue = 0.0;
    for(;i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++)
        instance->crtOptimalAction[i] = 0.5;

}


double* random_search_planning(random_search_instance* instance, unsigned int maxNbEvaluations) {

    double* optimalAction = (double*)malloc(sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);

    while(instance->crtNbEvaluations < maxNbEvaluations) {
        state* crt = instance->initial;
        state* next = NULL;
        double reward = 0.0;
        double firstAction[NUMBER_OF_DIMENSIONS_OF_ACTION];
        unsigned int crtDepth = 1;
        double discountedSum = 0.0;
        unsigned int i = 0;
        for(; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++)
            firstAction[i] = gsl_rng_uniform(instance->rng);

        nextStateReward(crt, firstAction, &next, &discountedSum);
        instance->crtNbEvaluations++;

        crt = next;

        while(crtDepth <= instance->crtDepthLimit) {
            double crtAction[NUMBER_OF_DIMENSIONS_OF_ACTION];
            char isTerminal = 0;
            for(i = 0; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++)
                crtAction[i] = gsl_rng_uniform(instance->rng);

            isTerminal = nextStateReward(crt, crtAction, &next, &reward) < 0 ? 1 : 0;
            instance->crtNbEvaluations++;
            discountedSum += instance->gammaPowers[crtDepth] * reward;

            freeState(crt);
            crt = next;

            if(isTerminal)
                break;

            crtDepth++;
        }

        freeState(crt);

        if(instance->crtDepthLimit > instance->crtMaxDepth)
            instance->crtMaxDepth = instance->crtDepthLimit;

        if(discountedSum > instance->crtOptimalValue) {
            instance->crtOptimalValue = discountedSum;
            memcpy(instance->crtOptimalAction, firstAction, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
        }

        if(instance->crtDepthLimit < (RANDOM_SEARCH_MAX_DEPTH - 1)) {
            instance->crtDepthLimit = h_max(instance);
            if(instance->crtDepthLimit >= RANDOM_SEARCH_MAX_DEPTH)
                instance->crtDepthLimit = RANDOM_SEARCH_MAX_DEPTH - 1;
            if(instance->crtDepthLimit < 1)
                instance->crtDepthLimit = 1;
        }
    }

    memcpy(optimalAction, instance->crtOptimalAction, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    return optimalAction;

}


unsigned int random_search_getMaxDepth(random_search_instance* instance) {

    return instance->crtMaxDepth - 1;

}


void random_search_uninitInstance(random_search_instance** instance) {

    gsl_rng_free((*instance)->rng);
    freeState((*instance)->initial);

    free(*instance);
    *instance = NULL;

}

#ifdef USE_SDL
void random_search_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance) {

    (void)screen;
    (void)screenWidth;
    (void)screenHeight;
    (void)instance;

}
#endif
