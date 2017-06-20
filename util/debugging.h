#ifndef DEBUGGING_H
#define DEBUGGING_H

#include "map/map.h"

bool dbg_set_contains(std::set<POI *> s, int dbg_id);
bool dbg_set_equals(std::set<POI *> s, int dbg_id1, int dbg_id2);

#endif
