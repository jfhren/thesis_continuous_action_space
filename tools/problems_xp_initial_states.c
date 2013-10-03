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
#include <argtable2.h>
#include <gsl/gsl_rng.h>
#include <time.h>
#include <string.h>
#include <math.h>


double* parseIntervals(const char* str, unsigned int* nbIntervals) {

    unsigned int maxNbIntervals = 16;
    double* intervals = (double*)malloc(sizeof(double) * maxNbIntervals * 2);

    unsigned int size = strlen(str);
    char* token = NULL;    
    char* tmp = (char*)malloc(sizeof(char) * (size + 1));
    memcpy(tmp, str, sizeof(char) * (size + 1));

    *nbIntervals = 0;
    token = strtok(tmp, ",");
    token++;
    while(token != NULL) {
        (*nbIntervals)++;

        if(*nbIntervals > maxNbIntervals) {
            maxNbIntervals += maxNbIntervals;
            intervals = realloc(intervals, sizeof(double) * maxNbIntervals * 2);
        }

        intervals[(*nbIntervals - 1) * 2] = strtod(token, NULL);
        token = strtok(NULL, "]");
        intervals[((*nbIntervals - 1) * 2 ) + 1] = strtod(token, NULL);

        token = strtok(NULL, "[");
        if(token != NULL)
            token = strtok(NULL, ",");        
    }

    intervals = realloc(intervals, sizeof(double) * *nbIntervals * 2);
    free(tmp);

    return intervals;

}

int main(int argc, char* argv[]) {

    unsigned int i = 0;
    unsigned int nbIntervals = 0;
    double* intervals = NULL;
    FILE* outputFileFd = NULL;
    gsl_rng* rng = NULL;
    unsigned int nbStates = 0;

    struct arg_file* outputFile = arg_file1("o", NULL, "<file>", "The output file for the generated initial state");
    struct arg_int* n = arg_int1("n", NULL, "<n>", "The number of initial states to generate");
    struct arg_str* s = arg_str1(NULL, "intervals", "<s>", "The intervals for the initial states generation");
    struct arg_end* end = arg_end(4);

    void* argtable[4];

    int nerrors = 0;

    argtable[0] = outputFile;
    argtable[1] = n;
    argtable[2] = s;
    argtable[3] = end;

    if(arg_nullcheck(argtable) != 0) {
        printf("error: insufficient memory\n");
        arg_freetable(argtable, 4);
        return EXIT_FAILURE;
    }

    nerrors = arg_parse(argc, argv, argtable);

    if(nerrors > 0) {
        printf("%s:", argv[0]);
        arg_print_syntax(stdout, argtable, "\n");
        arg_print_errors(stdout, end, argv[0]);
        arg_freetable(argtable, 4);
        return EXIT_FAILURE;
    }

    nbStates = n->ival[0];

    intervals = parseIntervals(s->sval[0], &nbIntervals);

    rng = gsl_rng_alloc(gsl_rng_mt19937);
    gsl_rng_set(rng, time(NULL));

    outputFileFd = fopen(outputFile->filename[0], "w");
    fprintf(outputFileFd, "%u\n", n->ival[0]);

    for(; i < nbStates; i++) {
        unsigned int j = 0;
        for(; j < (nbIntervals - 1); j++)
            fprintf(outputFileFd, "%.15f,", (fabs(intervals[(j * 2) + 1] - intervals[j * 2]) * gsl_rng_uniform(rng)) + intervals[j * 2]);
        fprintf(outputFileFd, "%.15f\n", (fabs(intervals[(j * 2) + 1] - intervals[j * 2]) * gsl_rng_uniform(rng)) + intervals[j * 2]);
    }

    fclose(outputFileFd);
	gsl_rng_free(rng);
    free(intervals);
    arg_freetable(argtable, 4);

    return EXIT_SUCCESS;

}
