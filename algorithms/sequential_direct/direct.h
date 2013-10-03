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

#ifndef DIRECT_H
#define DIRECT_H

typedef struct box_rec {
    double value;                                           /* The value associated with this box. */
    double centerPosition[NUMBER_OF_DIMENSIONS_OF_ACTION];  /* The center of this box.*/
    unsigned int level;                                     /* Used to compute the size of the box.*/
    unsigned int stage;                                     /* Used to compute the size of the box.*/
    struct box_rec* next;                                   /* Next box in this group.*/
    struct box_rec* prev;                                   /* Prev box in this group.*/
}   box;

typedef struct group_rec {
    double boxesSize;               /* Size of the boxes in this group.*/
    box* boxes;                     /* List of the boxes ordered by descending bound value.*/
    box* lastOne;                   /* The last box in the list of boxes.*/
    struct group_rec* next;         /* Next group. */
    struct group_rec* prev;         /* Previous group. */
    struct group_rec* nextPOGroup;  /* Next potentialy optimal group. */
}   group;

typedef struct {
    group* groupsList;          /* The list of the group order by descending boxe size. */
    group* pOGroups;            /* The list of currently potentialy optimal groups. */
    double crtMaxPOGroup;       /* The maximum value of the current POGroup being processed. Valid if pOGroup is not NULL. */
    box* boxesToBeAdded;        /* The boxes that will be added once a value has been given */
    group* groupToAddTheBoxes;  /* The group to which the boxes to be added should be */
    double crtMax;              /* The current maximu value within the direct algo. */
    double epsilon;             /* The precision. */
}   direct_algo;

direct_algo* direct_algo_init();
double* direct_algo_getAnAction(direct_algo* instance);
void direct_algo_updateValue(direct_algo* instance, double value);
void direct_algo_uninit(direct_algo** instance);

#endif
