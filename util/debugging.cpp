#include "debugging.h"

#include "config.h"

#ifdef ENABLE_DEBUG
bool dbg_set_contains(std::set<POI *> s, int dbg_id) {
  for (auto poi : s) {
    if (poi->getId() == dbg_id)
      return true;
  }
  return false;
}
bool dbg_set_equals(std::set<POI *> s, int dbg_id1, int dbg_id2) {
  std::set<int> dbg_set;
  for (auto poi : s) {
    dbg_set.insert(poi->getId());
  }

  return dbg_set == std::set<int>({dbg_id1, dbg_id2});
}
#else
bool dbg_set_contains(std::set<POI *> s, int dbg_id) {
  return false;
}
bool dbg_set_equals(std::set<POI *> s, int dbg_id1, int dbg_id2) {
  return false;
}
#endif
