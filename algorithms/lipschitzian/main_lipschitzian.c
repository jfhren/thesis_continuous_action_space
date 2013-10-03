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

#include "lipschitzian.h"


int main(int argc, char* argv[]) {

    double discountFactor;
    unsigned int maxNbEvaluations;
    char isTerminal = 0;
    int nbTimestep = -1;
    double L;

#ifdef USE_SDL
    char isDisplayed = 1;
    char verbose = 0;
    char resolution[255] = "640x480";
    char isFullscreen = 1;
#else
    char verbose = 1;
#endif

    lipschitzian_instance* instance = NULL;

    state* crtState = NULL;
    state* nextState = NULL;
    double reward = 0.0;
    double* optimalAction = NULL;

    struct arg_dbl* g = arg_dbl1("g", "discountFactor", "<d>", "The discount factor for the problem");
    struct arg_int* n = arg_int1("n", "nbEvaluations", "<n>", "The number of evaluations");
    struct arg_int* s = arg_int0("s", "nbtimestep", "<n>", "The number of timestep");
    struct arg_str* i = arg_str0(NULL, "state", "<s>", "The initial state to use");
    struct arg_dbl* l = arg_dbl1("L", NULL, "<d>", "The Lipschitz coefficient");

#ifdef USE_SDL
    struct arg_lit* d = arg_lit0("d", NULL, "Display the viewer");
    struct arg_lit* v = arg_lit0("v", NULL, "Verbose");
    struct arg_str* r = arg_str0(NULL, "resolution", "<s>", "The resolution of the display window");
    struct arg_lit* f = arg_lit0("f", NULL, "Fullscreen");
    void* argtable[10];
    int nbArgs = 9;
#else
    void* argtable[6];
    int nbArgs = 5;
#endif

    struct arg_end* end = arg_end(nbArgs+1);
    int nerrors = 0;

    s->ival[0] = -1;

    argtable[0] = g; argtable[1] = n; argtable[2] = l; argtable[3] = s; argtable[4] = i;

#ifdef USE_SDL
    argtable[5] = d;
    argtable[6] = v;
    argtable[7] = r;
    argtable[8] = f;
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
    L = l->dval[0];

    initGenerativeModelParameters();
    initGenerativeModel();
    if(i->count)
        crtState = makeState(i->sval[0]);
    else
        crtState = initState();

#ifdef USE_SDL
    if(r->count)
        strcpy(resolution, r->sval[0]);
    isDisplayed = d->count;
    verbose = v->count;
    isFullscreen = f->count;
#endif

    nbTimestep = s->ival[0];
    arg_freetable(argtable, nbArgs+1);

    instance = lipschitzian_initInstance(crtState, discountFactor, L);

#ifdef USE_SDL
    if(isDisplayed) {
        if(initViewer(resolution, lipschitzian_drawingProcedure, isFullscreen) == -1)
            return EXIT_FAILURE;
        viewer(crtState, NULL, 0.0, instance);
    }
#endif

    do {
        free(optimalAction);
        lipschitzian_resetInstance(instance, crtState);

        optimalAction = lipschitzian_planning(instance, maxNbEvaluations);

        isTerminal = nextStateReward(crtState, optimalAction, &nextState, &reward);
        freeState(crtState);
        crtState = nextState;

        if(verbose) {
            printState(crtState);
            printAction(optimalAction);
            printf("reward: %f mean depth: %f\n", reward, lipschitzian_getMeanDepth(instance));
        }
#ifdef USE_SDL
    } while(!isTerminal && (nbTimestep < 0 || --nbTimestep) && (isDisplayed || !viewer(crtState, optimalAction, reward, instance)));
#else
    } while(!isTerminal && (nbTimestep < 0 || --nbTimestep));
#endif

    free(optimalAction);
    freeState(crtState);

    lipschitzian_uninitInstance(&instance);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
