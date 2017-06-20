#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "map/map.h"

struct CarPosition
{
    Position pos;
    double angle;
    double zoom;
};

/* Abstract base class for all items in a trajectory
 */
class TrajectoryItem
{
public:
    static const int TYPE_INVALID = 0;
    static const int TYPE_CIRCLE = 1;
    static const int TYPE_STRAIGHT = 2;

    TrajectoryItem(double speed);
    virtual ~TrajectoryItem();

    /* Derived classes must implement this and return the correct TYPE_*
     * constant above.
     */
    virtual int getType();

    virtual CarPosition interpolatePosition(double dist) = 0;
    virtual double getLength() = 0;

    virtual double getStartZoom() = 0;
    virtual double getEndZoom() = 0;
    virtual double getStartAngle() = 0;
    virtual double getEndAngle() = 0;

    virtual void adjustZoomLevel(double zoom) = 0;

    double getSpeed();

    virtual void dbg_print() = 0;

private:
  double speed;
};

/* A circular part of the trajectory, i.e. a circle segment
 */
class CircleTrajectoryItem : public TrajectoryItem
{
public:
    /* Constructs a CircleTrajectoryItem
     *
     * Arguments:
     *  center:         The circle's center
     *  r:              The radius of the circle
     *  aStart:         The angle (taken from the center of the circle) at the start of the segment
     *  aEnd:           The angle (taken from the center of the circle) at the end of the segment
     *  zoom:           The zoom level throughout this rotation
     */
    CircleTrajectoryItem(Position center, double r, double aStart, double aEnd, double zoom, double speed);
    virtual int getType();

    double getAEnd() const;
    double getAStart() const;

    double getR() const;
    Position getCenter() const;

    CarPosition interpolatePosition(double dist);
    double getLength();

    double getStartZoom();
    double getEndZoom();
    double getStartAngle();
    double getEndAngle();

    void adjustZoomLevel(double zoom);

    void dbg_print();

private:
    Position center;
    double r;
    double aStart;
    double aEnd;
    double zoom;
};

/* A straight part of a trajectory, i.e. a line segment.
 */
class StraightTrajectoryItem : public TrajectoryItem
{
public:
    StraightTrajectoryItem(Position start, Position end, double startZoom, double endZoom, double speed);
    virtual int getType();

    Position getStart() const;
    Position getEnd() const;

    CarPosition interpolatePosition(double dist);
    double getLength();

    double getAngle();

    double getStartZoom();
    double getEndZoom();
    double getStartAngle();
    double getEndAngle();

    void adjustZoomLevel(double zoom);

    void dbg_print();
private:
    Position start;
    Position end;
    double startZoom;
    double endZoom;
};

class Trajectory
{
public:
    Trajectory();
    ~Trajectory();

    void addItem(TrajectoryItem *item);
    std::vector<TrajectoryItem*> getItems();
    CarPosition interpolatePosition(double dist);

    /**
     * BIG FAT WARNING: Thes dists must be sorted ascendingly!
     */
    std::vector<CarPosition> interpolatePositions(std::vector<double> dists);

    double getLength();

    void dbg_print();

private:
    std::vector<TrajectoryItem*> items;

// TrajectoryFactory needs to manipulate the items of this Trajectory directly..
friend class TrajectoryFactory;
friend class ZoomComputer;
};

#endif // TRAJECTORY_H
