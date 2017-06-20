#ifndef NOOVERLAPSCHECKER_H
#define NOOVERLAPSCHECKER_H

#include "heuristics/heuristic.h"
#include "conflicts/camera.h"
#include "map/map.h"
#include "map/trajectory.h"

class NoOverlapsChecker {
public:
  NoOverlapsChecker(Camera *camera, Map *map, Trajectory *trajectory);
  bool check();
  bool checkAt(int point, std::set<std::set<POI *>> *problems = nullptr);

private:
  Camera *camera;
  Map *map;
  Trajectory *trajectory;

  bool checkPair(const RotatingPOI &rpoi1, const RotatingPOI &rpoi2);

  std::map<std::set<POI *>, int> last_seen;
};

#endif
