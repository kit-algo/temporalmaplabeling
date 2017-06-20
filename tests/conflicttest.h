#ifndef CONFLICTTEST_H
#define CONFLICTTEST_H

#include "map/map.h"
#include "map/router.h"
#include "conflicts/camera.h"
#include "map/trajectoryfactory.h"

#include <random>

#define TEST_INCREMENT 1.0

class ConflictTest
{
public:
    ConflictTest(Map *map, int seed);
    void run(int iterations);

private:
    void test();
    void testConflicts(Trajectory *trajectory, Camera *camera);

    Map *map;
    std::mt19937 seed_rng;
    int seed;
    int route_seed;
};

#endif // CONFLICTTEST_H
