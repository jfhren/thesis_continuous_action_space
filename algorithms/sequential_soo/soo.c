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

#include "soo.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

soo* soo_init(unsigned int(*hMax)(unsigned int)) {

    unsigned int i = 0;
    soo* newInstance = (soo*)malloc(sizeof(soo));
    newInstance->list = (depth*)malloc(sizeof(depth));
    newInstance->list->depth = 0;
    newInstance->list->list = NULL;
    newInstance->list->last = NULL;
    newInstance->list->next = NULL;
    newInstance->list->prev = NULL;
    newInstance->crtMax = -DBL_MAX;
    newInstance->crtDepth = newInstance->list;
    newInstance->crtMaxValue = -DBL_MAX;
    newInstance->hMax = hMax;
    newInstance->t = 0;
    newInstance->depthToAddTheLeaves = newInstance->list;
    newInstance->leavesToBeAdded = (leaf*)malloc(sizeof(leaf));
    newInstance->leavesToBeAdded->value = 0.0;
    for(; i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++) {
        newInstance->crtMaxLeaf[i] = 0.5;
        newInstance->leavesToBeAdded->centerPosition[i] = 0.5;
    }
    newInstance->leavesToBeAdded->next = NULL;
    newInstance->leavesToBeAdded->prev = NULL;

    return newInstance;

}


double* soo_getAnAction(soo* instance) {

    if(instance->leavesToBeAdded == NULL) {
        unsigned int hMax = instance->hMax(instance->t);
        depth* crtDepth = instance->crtDepth;
        leaf* selectedLeaf = NULL;
        double shift = 0.0;
        unsigned int dimensionToCut = 0;

        while((crtDepth != NULL) && (crtDepth->depth <= hMax) && (crtDepth->list->value < instance->crtMax))
            crtDepth = crtDepth->next;

        if((crtDepth == NULL) || (crtDepth->depth > hMax))
            crtDepth = instance->list;

        selectedLeaf = crtDepth->list;
        dimensionToCut = crtDepth->depth % NUMBER_OF_DIMENSIONS_OF_ACTION;
        shift = 1.0 / pow(3, crtDepth->depth + 1);

        if(((crtDepth->next == NULL) || (crtDepth->next->depth != (crtDepth->depth + 1))) && (crtDepth->list->next == NULL)) {
            crtDepth->depth++;
            instance->depthToAddTheLeaves = crtDepth;
            instance->crtDepth = crtDepth;
            if(crtDepth->prev != NULL)
                instance->crtMax = crtDepth->prev->list->value;
            else
                instance->crtMax = -DBL_MAX;
        } else {
            crtDepth->list = selectedLeaf->next;
            if(crtDepth->list != NULL)
                crtDepth->list->prev = NULL;

            if((crtDepth->next == NULL) || (crtDepth->next->depth != (crtDepth->depth + 1))) {
                depth* newDepth = (depth*)malloc(sizeof(depth));
                newDepth->depth = crtDepth->depth+1;

                newDepth->prev = crtDepth;
                newDepth->next = crtDepth->next;
                crtDepth->next = newDepth;
                if(newDepth->next != NULL)
                    newDepth->next->prev = newDepth;

                newDepth->list = selectedLeaf;
                newDepth->last = selectedLeaf;
                selectedLeaf->next = NULL;
                selectedLeaf->prev = NULL;

                instance->depthToAddTheLeaves = newDepth;
                instance->crtDepth = newDepth;
                instance->crtMax = crtDepth->list->value;
            } else {
                /*
                leaf* crtLeaf = crtDepth->next->list;
                leaf* prevLeaf = NULL;

                while((crtLeaf != NULL) && (crtLeaf->value > selectedLeaf->value)) {
                    prevLeaf = crtLeaf;
                    crtLeaf = crtLeaf->next;
                }

                if(prevLeaf == NULL) {
                    crtDepth->next->list = selectedLeaf;
                    selectedLeaf->next = crtLeaf;
                } else { 
                    prevLeaf->next = selectedLeaf;
                    selectedLeaf->next = crtLeaf;
                }*/
                leaf* crtLeaf = crtDepth->next->last;
                leaf* nextLeaf = NULL;

                while((crtLeaf != NULL) && (crtLeaf->value < selectedLeaf->value)) {
                    nextLeaf = crtLeaf;
                    crtLeaf = crtLeaf->prev;
                }

                if(nextLeaf == NULL)
                    crtDepth->next->last = selectedLeaf;
                else
                    nextLeaf->prev = selectedLeaf;
                selectedLeaf->next = nextLeaf;

                if(crtLeaf == NULL)
                    crtDepth->next->list = selectedLeaf;
                else
                    crtLeaf->next = selectedLeaf;
                selectedLeaf->prev = crtLeaf;


                instance->depthToAddTheLeaves = crtDepth->next;
                instance->crtDepth = crtDepth->next;
                if(crtDepth->list == NULL) {
                    if(crtDepth->prev == NULL) {
                        instance->crtMax = -DBL_MAX;
                        instance->list = crtDepth->next;
                        instance->list->prev = NULL;
                    } else {
                        instance->crtMax = crtDepth->prev->list->value;
                        crtDepth->prev->next = crtDepth->next;
                        crtDepth->next->prev = crtDepth->prev;
                    }
                    free(crtDepth);
                } else {
                    instance->crtMax = crtDepth->list->value;
                }
            }
        }

        instance->leavesToBeAdded = (leaf*)malloc(sizeof(leaf));
        instance->leavesToBeAdded->value = 0.0;
        memcpy(instance->leavesToBeAdded->centerPosition, selectedLeaf->centerPosition, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
        instance->leavesToBeAdded->centerPosition[dimensionToCut] -= shift;
        instance->leavesToBeAdded->prev = NULL;

        instance->leavesToBeAdded->next = (leaf*)malloc(sizeof(leaf));
        instance->leavesToBeAdded->next->value = 0.0;
        memcpy(instance->leavesToBeAdded->next->centerPosition, selectedLeaf->centerPosition, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
        instance->leavesToBeAdded->next->centerPosition[dimensionToCut] += shift;
        instance->leavesToBeAdded->next->next = NULL;
        instance->leavesToBeAdded->next->prev = NULL;

    }

    return instance->leavesToBeAdded->centerPosition; 

}


void soo_updateValue(soo* instance, double value) {

    leaf* leafToAdd = instance->leavesToBeAdded;
/*    leaf* crtLeaf = instance->depthToAddTheLeaves->list;
    leaf* prevLeaf = NULL;

    leafToAdd->value = value;
    instance->leavesToBeAdded = leafToAdd->next;    
    instance->t++;

    while((crtLeaf != NULL) && (crtLeaf->value > leafToAdd->value)) {
        prevLeaf = crtLeaf;
        crtLeaf = crtLeaf->next;
    }

    if(prevLeaf == NULL) {
        leafToAdd->next = instance->depthToAddTheLeaves->list;
        instance->depthToAddTheLeaves->list = leafToAdd;
    } else {
        prevLeaf->next = leafToAdd;
        leafToAdd->next = crtLeaf;
    }
*/
    leaf* crtLeaf = instance->depthToAddTheLeaves->last;
    leaf* nextLeaf = NULL;

    leafToAdd->value = value;
    instance->leavesToBeAdded = leafToAdd->next;
    instance->t++;

    while((crtLeaf != NULL) && (crtLeaf->value < leafToAdd->value)) {
        nextLeaf = crtLeaf;
        crtLeaf = crtLeaf->prev;
    }

    if(nextLeaf == NULL)
        instance->depthToAddTheLeaves->last = leafToAdd;
    else
        nextLeaf->prev = leafToAdd;
    leafToAdd->next = nextLeaf;

    if(crtLeaf == NULL)
        instance->depthToAddTheLeaves->list = leafToAdd;
    else
        crtLeaf->next = leafToAdd;
    leafToAdd->prev = crtLeaf;

    if(leafToAdd->value > instance->crtMaxValue) {
        instance->crtMaxValue = leafToAdd->value;
        memcpy(instance->crtMaxLeaf, leafToAdd->centerPosition, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    }

}


void soo_uninit(soo** instance) {

    depth* crtDepth = (*instance)->list;

    while(crtDepth != NULL) {
        depth* nextDepth = crtDepth->next;
        leaf* crtLeaf = crtDepth->list;
        while(crtLeaf != NULL) {
            leaf* nextLeaf = crtLeaf->next;
            free(crtLeaf);
            crtLeaf = nextLeaf;
        }
        free(crtDepth);
        crtDepth = nextDepth;
    }

    if((*instance)->leavesToBeAdded != NULL){
        free((*instance)->leavesToBeAdded->next);
        free((*instance)->leavesToBeAdded);
    }

    free(*instance);
    *instance = NULL;

} 
