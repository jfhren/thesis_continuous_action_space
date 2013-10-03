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

#ifndef SOO_H
#define SOO_H

typedef struct leaf_rec {
    double value;
    double centerPosition[NUMBER_OF_DIMENSIONS_OF_ACTION];
    struct leaf_rec* next;
    struct leaf_rec* prev;
} leaf;

typedef struct depth_rec {
    unsigned int depth;
    leaf* list;
    leaf* last;
    struct depth_rec* next;
    struct depth_rec* prev;
} depth;

typedef struct {
    depth* list;
    depth* depthToAddTheLeaves;
    leaf* leavesToBeAdded;
    depth* crtDepth;
    double crtMax;
    double crtMaxLeaf[NUMBER_OF_DIMENSIONS_OF_ACTION];
    double crtMaxValue;
    unsigned int t;
    unsigned int(*hMax)(unsigned int);
} soo;

soo* soo_init(unsigned int(*hMax)(unsigned int));
double* soo_getAnAction(soo* instance);
void soo_updateValue(soo* instance, double value);
void soo_uninit(soo** instance);

#endif
