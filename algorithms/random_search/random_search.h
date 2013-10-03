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

#ifndef RANDOM_SEARCH
#define RANDOM_SEARCH

#include <gsl/gsl_rng.h>
#ifdef USE_SDL
    #include <SDL/SDL.h>
#endif

#include "../../problems/generative_model.h"

#ifdef LIMITED_DEPTH
#define RANDOM_SEARCH_MAX_DEPTH 512
#else
#define RANDOM_SEARCH_MAX_DEPTH 32768
#endif

typedef struct {

    state* initial;
    double gamma;
    double gammaPowers[RANDOM_SEARCH_MAX_DEPTH];

    gsl_rng* rng;
    unsigned int crtMaxDepth;

    unsigned int crtDepthLimit;
    unsigned int crtNbEvaluations;
    double crtOptimalValue;
    double crtOptimalAction[NUMBER_OF_DIMENSIONS_OF_ACTION];

}       random_search_instance;

extern unsigned int (*h_max)(random_search_instance*);

random_search_instance* random_search_initInstance(state* initial, double discountFactor);
void random_search_resetInstance(random_search_instance* instance, state* initial);
double* random_search_planning(random_search_instance* instance, unsigned int maxNbEvaluations);
void random_search_keepSubtree(random_search_instance* instance);
unsigned int random_search_getMaxDepth(random_search_instance* instance);
void random_search_uninitInstance(random_search_instance** instance);

#ifdef USE_SDL
void random_search_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance);
#endif

#endif
