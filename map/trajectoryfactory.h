#ifndef TRAJECTORYFACTORY_H
#define TRAJECTORYFACTORY_H

#include "trajectory.h"
#include "zoomcomputer.h"

#include <vector>

#include "map/map.h"

/* Helper class that constructs a trajectory out of a given route.
 */
class TrajectoryFactory {
public:
    /* Constructs a TrajectoryFactory
     *
     * Arguments:
     *  waypoints:          The route, given as a sequence of waypoints
     */
    TrajectoryFactory(std::vector<Position> * waypoints, std::vector<double> * speeds, double w, double h, Map *map);
    Trajectory *getTrajectory();
private:
  Trajectory *result;

  void build_unzoomed_trajectory();
  void zoom_trajectory();

  double w;
  double h;
  Map *map;

  std::vector<Position> * waypoints;
  std::vector<double> * speeds;
  CircleTrajectoryItem *compute_circle(Position cur, Position last, Position next, double len, double speed);
};


#endif
