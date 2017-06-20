#include "conflicttest.h"

#include <limits>
#include <QDebug>

#include "map/trajectory.h"
#include "config.h"

#include "util/coordinateutil.h"

#include <map>
#include <set>

ConflictTest::ConflictTest(Map *map, int seed):
    map(map), seed(seed)
{
    this->seed_rng = std::mt19937(seed);
}

void ConflictTest::run(int iterations)
{
    std::map<std::string, std::set<std::string>> filter = {
        {"amenity", {"parking"}}
    };
    this->map->set_poi_filter(filter);

    for (int i = 0; i < iterations ; i++) {
        this->test();
    }
}

void ConflictTest::test()
{
    std::uniform_int_distribution<int> uni(std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
    int iteration_seed = uni(this->seed_rng);
    std::cout << "========================================";
    std::cout << "|              TEST ROUND              |";
    std::cout << "========================================";
    std::cout << "Seed: " << iteration_seed;

    this->route_seed = iteration_seed;
    Router router(this->map, iteration_seed);
    std::vector<std::pair<edge_t, bool>> route = router.get_random_route();

    std::vector<Position> route_waypoints;
    std::vector<double> route_speeds;

    for (auto edge : route) {
        std::vector<Position> waypoints = this->map->get_waypoints(edge.first);
        route_waypoints.reserve(route_waypoints.size() + waypoints.size());
        route_speeds.insert(route_speeds.end(), waypoints.size(), this->map->get_way_speed(edge.first));

        if (edge.second) { // Traversing in edge direction..
            route_waypoints.insert(route_waypoints.end(), waypoints.begin(), waypoints.end());
        } else { // Traversing against edge direction
            route_waypoints.insert(route_waypoints.end(), waypoints.rbegin(), waypoints.rend());
        }
    }

    TrajectoryFactory tFac(&route_waypoints, &route_speeds, CAMERA_WIDTH, CAMERA_HEIGHT, this->map);
    Trajectory *trajectory = tFac.getTrajectory();
    Camera *camera = new Camera(CAMERA_WIDTH, CAMERA_HEIGHT,  trajectory, this->map, LABEL_FONT);
    camera->compute();

    this->testConflicts(trajectory, camera);

    delete camera;
    delete trajectory;
}

void ConflictTest::testConflicts(Trajectory *trajectory, Camera *camera)
{
    std::vector<std::pair<std::set<POI *>, double>> errors;

    for (double point = 0.0; point < trajectory->getLength(); point += TEST_INCREMENT) {
        std::set<std::set<POI *>> conflicts = camera->getConflictsAt(point);
        std::set<POI *> visible = camera->getVisibleAt(point);

        std::cout << "Testing " << this->route_seed << " @ " << point << " / " << trajectory->getLength() << " (" << visible.size() << " visible)";

        CarPosition carpos = trajectory->interpolatePosition(point);
        std::vector<RotatingPOI> rpois = camera->getPOIsForCar(carpos, true);

        for (auto rpoi1 : rpois) {
            for (auto rpoi2 : rpois) {
                if (rpoi1.getPoi() == rpoi2.getPoi()) {
                    continue;
                }

                if (CoordinateUtil::rpoiOverlap(&rpoi1, &rpoi2) &&
                        visible.find(rpoi1.getPoi()) != visible.end() &&
                        visible.find(rpoi2.getPoi()) != visible.end()) {
                    if (conflicts.find({rpoi1.getPoi(), rpoi2.getPoi()}) == conflicts.end()) {
                        errors.push_back({{rpoi1.getPoi(), rpoi2.getPoi()}, point});
                        std::cout << "Found error! Should be overlapping... " << rpoi1.getPoi()->getId() << " vs "  << rpoi2.getPoi()->getId() << " @ " << point;
                    }
                } else {
                    if (conflicts.find({rpoi1.getPoi(), rpoi2.getPoi()}) != conflicts.end()) {
                        errors.push_back({{rpoi1.getPoi(), rpoi2.getPoi()}, point});
                        std::cout << "Found error! Should not be overlapping... " << rpoi1.getPoi()->getId() << " vs "  << rpoi2.getPoi()->getId() << " @ " << point;
                    }
                }
            }
        }
    }

    if (errors.size() != 0) {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        std::cout << "|             Errors Found             |";
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        for (auto error : errors) {
            POI *poi1 = *(error.first.begin());
            POI *poi2 = *(++(error.first.begin()));
            double point = error.second;
            std::cout << poi1->getId() << " vs. " << poi2->getId() << " @ " << point;
        }
        exit(-1);
    }
}
