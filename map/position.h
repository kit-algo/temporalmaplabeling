#ifndef POSITION_H
#define POSITION_H

#include <QPair>

/* The "Position" datatype is the internal representation of any coordinate, except when stored
 * in the rtree.
 */
typedef QPair<double, double> Position;

#endif
