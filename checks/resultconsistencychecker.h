#ifndef RESULTCONSISTENCYCHECKER_H
#define RESULTCONSISTENCYCHECKER_H

#include "heuristics/heuristic.h"
#include "conflicts/camera.h"
#include "map/map.h"

class ResultConsistencyChecker {
public:
  ResultConsistencyChecker(SelectionIntervals *selected, Camera *camera, Map *map, int k);

  void check();

private:
  SelectionIntervals *selected;
  Camera *camera;
  Map *map;
  int k;

  void check_inside();
  void check_outside();
  void check_restriction();
  void prepare();

  bool triple_check(Interval i1, Interval i2, Interval i3);

  std::map<POI *, std::vector<Interval>> poi_selected;
  std::map<POI *, std::vector<Interval>> poi_visible;
  std::map<std::set<POI *>, std::vector<Interval>> poi_conflicts;
};


#endif
