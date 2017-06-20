#ifndef CAMERA_H
#define CAMERA_H

#include "map/map.h"
#include "map/projection.h"
#include "map/trajectory.h"
#include "map/trajectory.h"
#include "rotatingpoi.h"
#include "trajectoryfilter.h"

#include <QFont>

struct InterpolationStep {
  RotatingPOI viewport;
  CarPosition carpos;
  double dist;
};

class Camera
{
public:
    Camera(double width, double height, Trajectory *trajectory, Map *map, QFont font);

    void compute();

    std::vector<RotatingPOI> getPOIsForCar(CarPosition carpos, bool onlyVisible = false);
    CameraProjection getProjectionForCar(CarPosition carpos);

    double getWidth() const;

    double getHeight() const;

    std::vector<InterpolationStep> getInterpolationPoints();
    RotatingPOI getViewportAt(double dist);
    RotatingPOI getViewportAt(CarPosition carpos);

    std::set<std::set<POI *> > getConflictsAt(double dist);
    std::set<POI *> getVisibleAt(double dist);

    ConflictIntervals &getConflictIntervals();
    VisibilityIntervals &getVisibilityIntervals();
    ConflictIntervals &getDisplayPairs();

    double time_rotation_conflicts = -1;
    double time_path = -1;
    double time_zoom_conflicts = -1;
    double time_interpolate = -1;

private:
    double width;
    double height;
    Map *map;
    Trajectory *trajectory;

    QFont font;

    void precomputeRequirements();

    // Which zoom levels exist on a circular item, and where do they start and end? (level, start, end)
    std::set<std::tuple<double, double, double>> zoomLevels;
    // Which angles do straight items with zoom have, and where do they start and end? (angle, start, end)
    std::set<std::tuple<double, double, double>> zoomAngles;

    ConflictIntervals conflicts;

    ConflictIntervals displayPairs;

    VisibilityIntervals visibilityIntervals;

    void computeMaxLabelSpan();
    double maxLabelSpan;
};

#endif // CAMERA_H
