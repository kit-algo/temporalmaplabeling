#ifndef TRAJECTORYFILTER_H
#define TRAJECTORYFILTER_H

#include <utility>
#include <set>
#include <tuple>
#include "map/map.h"
#include "rotatingpoi.h"
#include "map/trajectory.h"
#include "util/setrtree.h"

struct DisplayPair {
    double start;
    double end;
    std::set<POI *>pois;
};

/*
#define bgi:: boost::geometry::index::

typedef boost::geometry::cs::cartesian coordinate_system_type;
typedef boost::geometry::model::point<double, 1, coordinate_system_type> Point;
typedef boost::geometry::model::segment<Point> Interval;
*/

//typedef boost::icl::interval<double>::type Interval;
//typedef std::pair<Interval, std::set<POI *>> ConflictInterval;

typedef SetRTree<std::set<POI *>> ConflictIntervals;
//typedef std::map<Interval, std::set<std::set<POI *>> ConflictMap;

//typedef boost::icl::interval_map<double, std::set<std::set<POI *>>> ConflictIntervalMap;

//typedef std::vector<DisplayPair> DisplayPairs;
typedef SetRTree<POI *> VisibilityIntervals;
//typedef std::map<Interval, std::set<POI *>> VisibilityMap;
//typedef boost::icl::interval_map<double, std::set<POI*>> VisibilityIntervals;
//typedef std::vector<std::tuple<double, double, POI*>> VisibilityIntervals;

class Camera;

class TrajectoryFilter
{
public:
    TrajectoryFilter(Trajectory *trajectory, Camera *camera, Map *map);

    ConflictIntervals getDisplayPairs();
    VisibilityIntervals getVisibilityIntervals() const;
    ConflictIntervals getConflicts();

private:
    Trajectory *trajectory;
    Camera *camera;
    Map *map;

    void computeVisiblePOI();

    std::set<POI *> visiblePOI;
    VisibilityIntervals visibilityIntervals;

    ConflictIntervals displayPairs;

    ConflictIntervals conflicts;
};

#endif // TRAJECTORYFILTER_H
