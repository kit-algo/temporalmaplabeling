#ifndef TRAJECTORYINTERPOLATOR_H
#define TRAJECTORYINTERPOLATOR_H

#include "trajectory.h"
#include "map.h"

class TrajectoryInterpolator
{
public:
    TrajectoryInterpolator(Trajectory *t, Map *m);

private:
    Trajectory *t;
    Map *m;
};

#endif // TRAJECTORYINTERPOLATOR_H
