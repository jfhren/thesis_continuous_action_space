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

#include "direct.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

double getBoxSize(unsigned int level, unsigned int stage) {
    return (sqrt(NUMBER_OF_DIMENSIONS_OF_ACTION - ((8.0 * stage ) / 9.0 )) / pow(3, level)) / 2.0;  /*l^2 diameter*/
}


/*Identify every potentially optimal groups*/

void identifyPOGroups(direct_algo* instance) {

    group* crt     = instance->groupsList->next;
    group* crtPrev = instance->groupsList;
    group* crtNext = NULL;
    group* crtLastPOGroup = NULL;

    if(instance->pOGroups != NULL)
        return;

    /*The biggest box with with the biggest value of his group is potentially optimal*/
    instance->pOGroups = instance->groupsList;
    instance->pOGroups->nextPOGroup = NULL;
    instance->crtMaxPOGroup = instance->pOGroups->boxes->value;
    crtLastPOGroup = instance->pOGroups;

    /*If there isn't more than one box*/
    if(crt == NULL)
        return;

    while(crt != NULL) {
        double hl;
        double ll;

        /*Evaluating the high limit of eq. 2.29 of Lemma 2.4 of Gablonsky's Thesis*/
        hl = (crt->boxes->value - crtPrev->boxes->value) / (crtPrev->boxesSize - crt->boxesSize);

        crtPrev = crtPrev->prev;
        while(crtPrev != NULL) {
            double l = (crt->boxes->value - crtPrev->boxes->value) / (crtPrev->boxesSize - crt->boxesSize);

            if(l < hl)
                hl = l;

            crtPrev = crtPrev->prev;
        }

        /*Evaluating the low limit of eq. 2.29 of Lemma 2.4 of Gablonsky's Thesis*/
        crtNext = crt->next;

        if(crtNext != NULL) {
            ll = (crtNext->boxes->value - crt->boxes->value) / (crt->boxesSize - crtNext->boxesSize);
            crtNext = crtNext->next;

            while(crtNext != NULL) {
                double l = (crtNext->boxes->value - crt->boxes->value) / (crt->boxesSize - crtNext->boxesSize);

                if(l > ll) {
                    ll = l;
                }

                crtNext = crtNext->next;
            }
        } else {
            ll = hl;
        }

        /*Verify eq. 2.29 of Lemma 2.4 of Gablonsky's Thesis*/

        if(ll <= hl) {

            if(instance->crtMax != 0) {
                /*Verify eq. 2.30 of Lemma 2.4 of Gablonsky's Thesis*/
                if((instance->epsilon * fabs(instance->crtMax)) <= ((crt->boxes->value - instance->crtMax) + (crt->boxesSize * hl))) {
                    crtLastPOGroup->nextPOGroup = crt;
                    crt->nextPOGroup = NULL;
                    crtLastPOGroup = crt;
                }
            } else {
                /*Verify eq. 2.31 of Lemma 2.4 of Gablonsky's Thesis*/
                if(crt->boxes->value <= (crt->boxesSize * hl)) {
                    crtLastPOGroup->nextPOGroup = crt;
                    crt->nextPOGroup = NULL;
                    crtLastPOGroup = crt;
                }
            }
        }

        crtPrev = crt;
        crt = crt->next;

    }

}


/* Add a box to a group */

void addBoxToGroup(group* g, box* b) {

    if(g == NULL)
        return;

    if(g->boxes == NULL) {
        g->boxes = b;
        b->next = NULL;
        b->prev = NULL;
        g->lastOne = b;
    } else {
        box* crtBox = g->lastOne;

        while((crtBox != NULL) && (crtBox->value < b->value))
            crtBox = crtBox->prev;
        if(crtBox != NULL) {
            if(crtBox->next != NULL)
                crtBox->next->prev = b;
            else
                g->lastOne = b;

            b->next = crtBox->next;
            b->prev = crtBox;

            crtBox->next = b;
        } else {
            b->next = g->boxes;
            b->prev = NULL;
            b->next->prev = b;
            g->boxes = b;
        }
    }

}


/* Divide a potentially optimal box */

void dividePOBoxes(direct_algo* instance) {

    box* crt = instance->pOGroups->boxes;
    group* modifiedGroup = instance->pOGroups;
    group* crtGroup   = modifiedGroup->next;
    group* prevGroup  = modifiedGroup;

    double shift     = 1.0 / pow(3, crt->level + 1);
    unsigned int dimensionToDivide = crt->stage++;
    double boxSize = 0.0;

    if(crt->stage == NUMBER_OF_DIMENSIONS_OF_ACTION) {
        crt->stage = 0;
        crt->level++;
    }

    boxSize = getBoxSize(crt->level, crt->stage);

    while((crtGroup!= NULL) && (crtGroup->boxesSize > boxSize)) {
        prevGroup = crtGroup;
        crtGroup = crtGroup->next;
    }

    if((crtGroup == NULL) || (crtGroup->boxesSize < boxSize)) {
        if(crt->next == NULL) {
            instance->pOGroups = modifiedGroup->nextPOGroup;
            if(instance->pOGroups != NULL)
                instance->crtMaxPOGroup = instance->pOGroups->boxes->value;

            modifiedGroup->boxes = crt;
            modifiedGroup->lastOne = crt;
            modifiedGroup->boxesSize = boxSize;
            modifiedGroup->next = crtGroup;
            if(prevGroup != modifiedGroup)
                modifiedGroup->prev = prevGroup;
            else
                modifiedGroup->prev = NULL;
            modifiedGroup->nextPOGroup = NULL;

            if(modifiedGroup->prev == NULL)
                instance->groupsList = modifiedGroup;
            else
                modifiedGroup->prev->next = modifiedGroup;

            if(crtGroup != NULL)
                crtGroup->prev = modifiedGroup;

            instance->groupToAddTheBoxes = modifiedGroup;
        } else {
            group* newGroup = (group*)malloc(sizeof(group));
            newGroup->boxesSize = boxSize;
            modifiedGroup->boxes = crt->next;
            modifiedGroup->boxes->prev = NULL;
            newGroup->boxes = crt;
            newGroup->lastOne = crt;
            crt->next = NULL;
            crt->prev = NULL;
            newGroup->next = crtGroup;
            newGroup->prev = prevGroup;
            newGroup->nextPOGroup = NULL;

            if(crtGroup != NULL)
                crtGroup->prev = newGroup;
            prevGroup->next = newGroup;

            instance->groupToAddTheBoxes = newGroup;
            if(instance->pOGroups->boxes->value != instance->crtMaxPOGroup) {
                instance->pOGroups = modifiedGroup->nextPOGroup;
                if(instance->pOGroups != NULL)
                    instance->crtMaxPOGroup = instance->pOGroups->boxes->value;
            }
        }
    } else {
        if(crt->next == NULL) {
            instance->pOGroups = modifiedGroup->nextPOGroup;
            if(instance->pOGroups != NULL)
                instance->crtMaxPOGroup = instance->pOGroups->boxes->value;
            if(modifiedGroup->prev != NULL)
                modifiedGroup->prev->next = modifiedGroup->next;
            else
                instance->groupsList = modifiedGroup->next;
            if(modifiedGroup->next != NULL)
                modifiedGroup->next->prev = modifiedGroup->prev;
            free(modifiedGroup);
        } else {
            modifiedGroup->boxes = crt->next;
            modifiedGroup->boxes->prev = NULL;
            crt->next = NULL;
            crt->prev = NULL;
            if(instance->pOGroups->boxes->value != instance->crtMaxPOGroup) {
                instance->pOGroups = instance->pOGroups->nextPOGroup;
                if(instance->pOGroups != NULL)
                    instance->crtMaxPOGroup = instance->pOGroups->boxes->value;
            }
        }
        addBoxToGroup(crtGroup, crt);
        instance->groupToAddTheBoxes = crtGroup;
    }

    /* The first new box */

    instance->boxesToBeAdded = (box*)malloc(sizeof(box));
    memcpy(instance->boxesToBeAdded->centerPosition, crt->centerPosition, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    instance->boxesToBeAdded->centerPosition[dimensionToDivide] += shift;                        /*New sample point*/
    instance->boxesToBeAdded->level = crt->level;
    instance->boxesToBeAdded->stage = crt->stage;
    instance->boxesToBeAdded->prev = NULL;

    /* the second new box */

    instance->boxesToBeAdded->next = (box*)malloc(sizeof(box));
    memcpy(instance->boxesToBeAdded->next->centerPosition, crt->centerPosition, sizeof(double) * NUMBER_OF_DIMENSIONS_OF_ACTION);
    instance->boxesToBeAdded->next->centerPosition[dimensionToDivide] -= shift;                  /*New sample point*/
    instance->boxesToBeAdded->next->level = crt->level;
    instance->boxesToBeAdded->next->stage = crt->stage;
    instance->boxesToBeAdded->next->next = NULL;
    instance->boxesToBeAdded->next->prev = NULL;

}


/* Init the algorithme */

direct_algo* direct_algo_init() {

    unsigned int i = 0;

    direct_algo* newInstance = (direct_algo*)malloc(sizeof(direct_algo));

    newInstance->groupsList = (group*)malloc(sizeof(group));
    newInstance->groupsList->boxesSize = getBoxSize(0, 0);
    newInstance->groupsList->boxes = (box*)malloc(sizeof(box));
    for(;i < NUMBER_OF_DIMENSIONS_OF_ACTION; i++) {
        newInstance->groupsList->boxes->centerPosition[i] = 0.5;
    }

    newInstance->groupsList->boxes->level = 0;
    newInstance->groupsList->boxes->stage = 0;
    newInstance->groupsList->boxes->next = NULL;
    newInstance->groupsList->boxes->prev = NULL;
    newInstance->groupsList->lastOne = newInstance->groupsList->boxes;
    newInstance->groupsList->next = NULL;
    newInstance->groupsList->prev = NULL;
    newInstance->groupsList->nextPOGroup = NULL;
    newInstance->pOGroups = NULL;
    newInstance->boxesToBeAdded = newInstance->groupsList->boxes;
    newInstance->groupToAddTheBoxes = NULL;
    newInstance->epsilon = 0.001;
    newInstance->crtMax = 0.0;

    return newInstance;

}


/* Return the selected box's center */

double* direct_algo_getAnAction(direct_algo* instance) {
    if(instance->boxesToBeAdded == NULL) {
        if(instance->pOGroups == NULL)
            identifyPOGroups(instance);
        dividePOBoxes(instance);
    }

    return instance->boxesToBeAdded->centerPosition;
}


/* update the selected box with a value */

void direct_algo_updateValue(direct_algo* instance, double value) {
    box* boxToUpdate = instance->boxesToBeAdded;
    boxToUpdate->value = value;
    if(value > instance->crtMax)
        instance->crtMax = value;
    instance->boxesToBeAdded = boxToUpdate->next;
    addBoxToGroup(instance->groupToAddTheBoxes, boxToUpdate);
}


/* Free up the instance */

void direct_algo_uninit(direct_algo** instance) {
    group* crtGroup = (*instance)->groupsList;
    while(crtGroup != NULL) {
        group* nextGroup = crtGroup->next;
        box* crtBox = crtGroup->boxes;
        while(crtBox != NULL) {
            box* nextBox = crtBox->next;
            free(crtBox);
            crtBox = nextBox;
        }
        free(crtGroup);
        crtGroup = nextGroup;
    }

    if(((*instance)->boxesToBeAdded != NULL) && ((*instance)->groupToAddTheBoxes != NULL)) {
        free((*instance)->boxesToBeAdded->next);
        free((*instance)->boxesToBeAdded);
    }

    free(*instance);
    *instance = NULL;

}
