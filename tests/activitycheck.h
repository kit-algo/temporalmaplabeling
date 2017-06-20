#ifndef ACTIVITYCHECK_H
#define ACTIVITYCHECK_H

#include "../heuristics/heuristic.h"

#include "conflicts/conflictgraph.h"


/**
 * Checks whether the activity is valid with respect to the given model:
 * Values for model:
 * 1: AM1
 * 2: AM2
 * 3: AM3
 * Checks that:
 * - no activities are in conflict.
 * - activity repsects the given model.
 */
bool checkActivity(const SelectionIntervals & activity, const ExpandedConflictGraph & g, int model);

#endif // ACTIVITYCHECK_H
