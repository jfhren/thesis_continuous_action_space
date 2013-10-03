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
#include <unistd.h>
#include <argtable2.h>

#include "../../problems/generative_model.h"
#ifdef USE_SDL
    #include "../../problems/viewer.h"
#endif
#include "random_search.h"

int main(int argc, char* argv[]) {

    double discountFactor;
    unsigned int maxNbEvaluations;
    char isTerminal = 0;
    int nbTimestep = -1;

    random_search_instance* instance = NULL;

    state* crtState = NULL;
    state* nextState = NULL;
    double reward = 0.0;
    double* optimalAction = NULL;

#ifdef USE_SDL
    char isDisplayed = 1;
    char verbose = 0;
    char resolution[255] = "640x480";
    char isFullscreen = 1;
#else
    char verbose = 1;
#endif

    struct arg_dbl* g = arg_dbl1("g", "discountFactor", "<d>", "The discount factor for the problem");
    struct arg_int* n = arg_int1("n", "nbEvaluations", "<n>", "The number of evaluations");
    struct arg_int* s = arg_int0("s", "nbtimestep", "<n>", "The number of timestep");

#ifdef USE_SDL
    struct arg_lit* d = arg_lit0("d", NULL, "Display the viewer");
    struct arg_lit* v = arg_lit0("v", NULL, "Verbose");
    struct arg_str* r = arg_str0(NULL, "resolution", "<s>", "The resolution of the display window");
    struct arg_lit* f = arg_lit0("f", NULL, "Fullscreen");
    void* argtable[9];
    int nbArgs = 8;
#else
    void* argtable[4];
    int nbArgs = 3;
#endif

    struct arg_end* end = arg_end(nbArgs+1);
    int nerrors = 0;

    s->ival[0] = -1;

    argtable[0] = g; argtable[1] = n; argtable[2] = s;

#ifdef USE_SDL
    argtable[3] = d;
    argtable[4] = v;
    argtable[5] = r;
    argtable[6] = f;
#endif

    argtable[nbArgs] = end;


    if(arg_nullcheck(argtable) != 0) {
        printf("error: insufficient memory\n");
        arg_freetable(argtable, nbArgs+1);
        return EXIT_FAILURE;
    }

    nerrors = arg_parse(argc, argv, argtable);

    if(nerrors > 0) {
        printf("%s:", argv[0]);
        arg_print_syntax(stdout, argtable, "\n");
        arg_print_errors(stdout, end, argv[0]);
        arg_freetable(argtable, nbArgs+1);
        return EXIT_FAILURE;
    }

    discountFactor = g->dval[0];
    maxNbEvaluations = n->ival[0];

    initGenerativeModelParameters();
    initGenerativeModel();
    crtState = initState();
    nbTimestep = s->ival[0];

#ifdef SDL_USE
    isDisplayed = d->count;
    verbose = v->count;
    isFullscreen = f->count;
    if(r->count)
        strcpy(resolution, r->sval[0]);
#endif

    arg_freetable(argtable, nbArgs+1);

    instance = random_search_initInstance(crtState, discountFactor);

#ifdef USE_SDL
    if(isDisplayed) {    
        if(initViewer(resolution, NULL, isFullscreen) == -1)
            return EXIT_FAILURE;
        viewer(crtState, NULL, 0.0, instance);
    }
#endif

    do {
        random_search_resetInstance(instance, crtState);

        optimalAction = random_search_planning(instance, maxNbEvaluations);

        isTerminal = nextStateReward(crtState, optimalAction, &nextState, &reward);
        freeState(crtState);
        crtState = nextState;

        if(verbose) {
            printState(crtState);
            printAction(optimalAction);
            printf("Reward: %f depth: %u\n", reward, random_search_getMaxDepth(instance));
        }
#ifdef USE_SDL
    } while(!isTerminal && (nbTimestep < 0 || --nbTimestep) && (!isDisplayed || !viewer(crtState, optimalAction, reward, instance)));
#else
    } while(!isTerminal && (nbTimestep < 0 || --nbTimestep));
#endif

    freeState(crtState);

    random_search_uninitInstance(&instance);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
