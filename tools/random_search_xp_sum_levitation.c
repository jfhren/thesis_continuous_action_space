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
#include <time.h>
#include <argtable2.h>
#include <math.h>
#include <string.h>

#include "../algorithms/random_search/random_search.h"

#include "../problems/levitation/levitation.h"

static unsigned int crtDepth = 1;

unsigned int h_max_crt_depth(random_search_instance* instance) {

    (void)instance;
    return crtDepth;

}

unsigned int* parseUnsignedIntList(char* str, unsigned int* nbItems) {

    unsigned int maxNbItems = 16;
    unsigned int* list = (unsigned int*)malloc(sizeof(unsigned int) * maxNbItems);

    unsigned int size = strlen(str);
    char* token = NULL;
    char* tmp = (char*)malloc(sizeof(char) * (size + 1));
    memcpy(tmp, str, sizeof(char) * (size + 1));

    *nbItems = 0;
    token = strtok(tmp, ",");
    while(token != NULL) {
        (*nbItems)++;

        if(*nbItems > maxNbItems) {
            maxNbItems += maxNbItems;
            list = realloc(list, sizeof(unsigned int) * maxNbItems);
        }

        list[(*nbItems - 1)] = strtol(token, NULL, 10);
        token = strtok(NULL, ",");        
    }

    list = realloc(list, sizeof(unsigned int) * *nbItems);
    free(tmp);

    return list;

}


int main(int argc, char* argv[]) {

    double discountFactor = 0.9;

    FILE* initFileFd = NULL;
    double* setPoints = NULL;
    FILE* results = NULL;
    char str[1024];
    unsigned int i = 0;
    unsigned int nbSetPoints = 0;
    unsigned int* ns = NULL;
    unsigned int nbN = 0;
    unsigned int* hs = NULL;
    unsigned int nbH = 0;
    unsigned int nbSteps = 0;
    unsigned int nbIterations = 0;
    unsigned int timestamp = time(NULL);
    int readFscanf = -1;
    random_search_instance* random_search = NULL;

    struct arg_file* initFile = arg_file1(NULL, "init", "<file>", "File containing the set points");
    struct arg_int* s = arg_int1("s", NULL, "<n>", "Number of steps");
    struct arg_int* it = arg_int1("i", NULL, "<n>", "Number of iterations");
    struct arg_str* r = arg_str1("n", NULL, "<s>", "List of ressources");
    struct arg_str* d = arg_str1("h", NULL, "<s>", "List of depth");
    struct arg_file* where = arg_file1(NULL, "where", "<file>", "Directory where we save the outputs");
    struct arg_end* end = arg_end(7);

    int nerrors = 0;
    void* argtable[7];

    argtable[0] = initFile;
    argtable[1] = r;
    argtable[2] = d;
    argtable[3] = s;
    argtable[4] = it;
    argtable[5] = where;
    argtable[6] = end;

    if(arg_nullcheck(argtable) != 0) {
        printf("error: insufficient memory\n");
        arg_freetable(argtable, 7);
        return EXIT_FAILURE;
    }

    nerrors = arg_parse(argc, argv, argtable);

    if(nerrors > 0) {
        printf("%s:", argv[0]);
        arg_print_syntax(stdout, argtable, "\n");
        arg_print_errors(stdout, end, argv[0]);
        arg_freetable(argtable, 7);
        return EXIT_FAILURE;
    }

    initGenerativeModelParameters();
    initGenerativeModel();

    initFileFd = fopen(initFile->filename[0], "r");
    readFscanf = fscanf(initFileFd, "%u\n", &nbSetPoints);
    setPoints = (double*)malloc(sizeof(double) * nbSetPoints);
    
    for(; i < nbSetPoints; i++) {
        readFscanf = fscanf(initFileFd, "%s\n", str);
        setPoints[i] = strtod(str, NULL);
    }
    fclose(initFileFd);

    nbSteps = s->ival[0];
    nbIterations = it->ival[0];
    ns = parseUnsignedIntList((char*)r->sval[0], &nbN);
    hs = parseUnsignedIntList((char*)d->sval[0], &nbH);

    sprintf(str, "%s/%u_results_random_search_%s.csv", where->filename[0], timestamp, r->sval[0]);
    results = fopen(str, "w");

    random_search = random_search_initInstance(NULL, discountFactor);
    h_max = h_max_crt_depth;

    for(i = 0; i < nbIterations; i++) {
        unsigned int j = 0;

        for(;j < nbH; j++) {
            unsigned int k = 0;

            crtDepth = hs[j];

            for(; k < nbN; k++) {                                          /* Loop on the computational ressources */
                state* crt = initState();
                double average = 0.0;
                unsigned int l = 0;

                printf("Starting with %u computational ressources\n", ns[k]);
                fprintf(results, "%u,%u", hs[j], ns[k]);
                fflush(NULL);

                for(; l < nbSetPoints; l++) {                           /* Loop on the initial states */
                    unsigned int m = 0;
                    parameters[10] = setPoints[l];

                    for(; m < nbSteps; m++) {                               /* Loop on the step */
                        char isTerminal = 0;
                        double reward = 0.0;
                        state* nextState = NULL;
                        random_search_resetInstance(random_search, crt);

                        double* optimalAction = random_search_planning(random_search, ns[k]);
                        isTerminal = nextStateReward(crt, optimalAction, &nextState, &reward) < 0 ? 1 : 0;
                        freeState(crt);
                        crt = nextState;
                        average += reward;
                        if(isTerminal)
                            break;
                    }

                    random_search_resetInstance(random_search, crt);
                    printf("random search: %u set point done\n", l);
                    fflush(NULL);
                }

                freeState(crt);
                average = average /(double)nbSetPoints;
                printf("Computation with %u computational ressources done\n\n", ns[k]);
                fprintf(results, ",%.15f\n", average);
                fflush(NULL);
            }

            printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>Depth %u done\n", hs[j]);
            fflush(NULL);
        }

        printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ITERATION %u DONE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n",i+1);
        fprintf(results,"\n");
        fflush(NULL);
    }
 
    fclose(results);

    arg_freetable(argtable, 7);

    free(setPoints);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
