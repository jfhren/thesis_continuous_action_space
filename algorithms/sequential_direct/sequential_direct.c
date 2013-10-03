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

#include "sequential_direct.h"

#include <stdlib.h>
#include <string.h>

#ifdef USE_SDL
    #include <SDL/SDL.h>
    #include <SDL/SDL_gfxPrimitives.h>
#endif

#include "../../problems/generative_model.h"
#include "direct.h"


sequential_direct_instance* sequential_direct_initInstance(state* initial, double gamma, unsigned int H, char dropTerminal) {

    unsigned int i = 0;
    sequential_direct_instance* newInstance = (sequential_direct_instance*)malloc(sizeof(sequential_direct_instance));
    newInstance->H = H;
    newInstance->dropTerminal = dropTerminal;
    newInstance->initial = copyState(initial);
    newInstance->instances = (direct_algo**)malloc(sizeof(direct_algo*) * H);
    for(;i < H; i++)
        newInstance->instances[i] = direct_algo_init();
    newInstance->gamma = gamma;
    for(i = 0; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++)
        newInstance->crtOptimalAction[i] = 0.5;
    newInstance->crtMaxSumOfDiscountedRewards = 0.0;
    newInstance->rewards = (double*)malloc(sizeof(double) * H);
    newInstance->crtNbEvaluations = 0;
    return newInstance;

}


static void buildTrajectory(sequential_direct_instance* instance) {

    unsigned int i = 1;
    state* crtState = NULL;
    double q = 0;
    double* action = direct_algo_getAnAction(instance->instances[0]);
    double* firstAction = action;
    char isTerminal = nextStateReward(instance->initial, action, &crtState, instance->rewards) < 0 ? 1 : 0;
    instance->crtNbEvaluations++;

    while((i < instance->H) && !(instance->dropTerminal && isTerminal)) {
        state* nextState = NULL;
        action = direct_algo_getAnAction(instance->instances[i]);
        isTerminal = nextStateReward(crtState, action, &nextState, instance->rewards + i) < 0 ? 1 : 0;
        instance->crtNbEvaluations++;
        freeState(crtState);
        crtState = nextState;
        i++;
    }
    freeState(crtState);

    for(; i > 0; i--) {
        q = instance->rewards[i-1] + (instance->gamma * q);
        direct_algo_updateValue(instance->instances[i-1], q);
    }

    if(q > instance->crtMaxSumOfDiscountedRewards) {
        instance->crtMaxSumOfDiscountedRewards = q;
        memcpy(instance->crtOptimalAction, firstAction, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    }
}


double* sequential_direct_planning(sequential_direct_instance* instance, unsigned int maxNbEvaluations) {

    double* optimalAction = (double*)malloc(sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);

    while(instance->crtNbEvaluations < maxNbEvaluations)
        buildTrajectory(instance);

    memcpy(optimalAction, instance->crtOptimalAction, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    return optimalAction;

}


void sequential_direct_uninitInstance(sequential_direct_instance** instance) {

    unsigned int i = 0;
    for(;i < (*instance)->H; i++)
        direct_algo_uninit((*instance)->instances+i);
    free((*instance)->instances);
    freeState((*instance)->initial);
    free((*instance)->rewards);
    free((*instance));
    *instance = NULL;

}

#ifdef USE_SDL
void sequential_direct_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance) {

    #if NUMBER_OF_DIMENSIONS_OF_ACTION == 1
    sequential_direct_instance* theInstance = (sequential_direct_instance*)instance;
    if(theInstance != NULL) {
        unsigned int H = theInstance->H;
        unsigned int i = 0;
        unsigned int heightShift = (H == 1) ? 0 : (screenHeight-20)/(H-1);
        for(; i < H; i++) {
            group* crtGroup = theInstance->instances[i]->groupsList;
            while(crtGroup != NULL) {
                box* crtBox = crtGroup->boxes;
                double y = 10 + (i * heightShift);
                aalineRGBA(screen, (screenWidth / 2.0) + 10, y, screenWidth - 10, y, 0, 0, 0, 255);
                while(crtBox != NULL) {
                    double x = (screenWidth / 2.0) + 10 + (crtBox->centerPosition[0] * ((screenWidth / 2.0) - 20));
                    aalineRGBA(screen, x, y + 2, x, y - 2, 0, 0, 0, 255);
                    crtBox = crtBox->next;
                }
                crtGroup = crtGroup->next;
            }
        }
    }
    #else
    (void)screen;
    (void)screenWidth;
    (void)screenHeight;
    (void)instance;
    #endif

}
#endif
