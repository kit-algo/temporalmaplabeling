#include "trajectory.h"

#include "util/coordinateutil.h"

#include <math.h>
#include <algorithm>
#include <armadillo>
#include <assert.h>

#include <QDebug>

#define DELTA 0.01

Trajectory::Trajectory()
{
}

Trajectory::~Trajectory()
{
    for (TrajectoryItem *i : this->items) {
        delete i;
    }
}

void Trajectory::dbg_print() {
  double dist = 0.0;
  for (auto item : this->items) {
    std::cout << "Distance " << dist << ": ";
    item->dbg_print();
    dist += item->getLength();
  }
}

void StraightTrajectoryItem::dbg_print()
{
  std::cout << "STI: " << this->start.first << "," << this->start.second << " (@" << this->startZoom << ") -> " << this->end.first << "," << this->end.second << " (@" << this->endZoom << ") (L: " << this->getLength() << ", A: " << this->getStartAngle() << ")\n";
}

void CircleTrajectoryItem::dbg_print()
{
  std::cout << "CTI: " << this->getStartAngle() << " -> "  << this->getEndAngle() << " (L: " << this->getLength() << ", Z: " << this->getStartZoom() << ")\n";
}

void Trajectory::addItem(TrajectoryItem *item)
{
    this->items.push_back(item);
}

std::vector<TrajectoryItem *> Trajectory::getItems()
{
    return this->items;
}

std::vector<CarPosition>
Trajectory::interpolatePositions(std::vector<double> dists)
{
  double distDone = 0.0;
  size_t cur_dist_index = 0;
  std::vector<CarPosition> res;

  for (auto item : this->getItems()) {
      while ((cur_dist_index < dists.size()) && (distDone + item->getLength() >= dists[cur_dist_index])) {
        res.push_back(item->interpolatePosition(dists[cur_dist_index] - distDone));
        cur_dist_index++;
      }
      distDone += item->getLength();
  }
  if ((dists.size() == (res.size() + 1)) && ((distDone + DELTA) <= dists[cur_dist_index])) {
    auto lastItem = this->getItems()[this->getItems().size() - 1];
    res.push_back(lastItem->interpolatePosition(lastItem->getLength()));
  }

  assert(res.size() == dists.size());
  return res;
}

CarPosition Trajectory::interpolatePosition(double dist)
{
    double distDone = 0.0;

    for (auto item : this->getItems()) {
        if (distDone + item->getLength() >= dist) {
            return item->interpolatePosition(dist - distDone);
        }
        distDone += item->getLength();
    }
    if ((distDone + DELTA) <= dist) {
      auto lastItem = this->getItems()[this->getItems().size() - 1];
      return lastItem->interpolatePosition(lastItem->getLength());
    }
    throw "Invalid distance!";
}

double Trajectory::getLength()
{
    double dist = 0.0;
    for (auto item : this->getItems()) {
        dist += item->getLength();
    }
    return dist;
}

int CircleTrajectoryItem::getType() {
    return TrajectoryItem::TYPE_CIRCLE;
}
double CircleTrajectoryItem::getAEnd() const
{
    return aEnd;
}
double CircleTrajectoryItem::getAStart() const
{
    return aStart;
}
double CircleTrajectoryItem::getR() const
{
    return r;
}
Position CircleTrajectoryItem::getCenter() const
{
    return center;
}

CarPosition CircleTrajectoryItem::interpolatePosition(double dist)
{
    assert(dist < this->getLength() + DELTA);
    assert(dist >= 0);
    if (dist > this->getLength()) {
        dist = this->getLength();
    }

    double length = this->getLength();
    double frac = dist / length;

    double dangle = CoordinateUtil::angleDiff(this->getAStart(), this->getAEnd());
    bool bendLeft = (dangle > 0);

    double resAngle = this->getAStart() + (dangle * frac);

    if (resAngle < -1*M_PI)
        resAngle += 2*M_PI;
    if (resAngle > M_PI)
        resAngle -= 2*M_PI;

    double dx = -1 * sin(resAngle) * this->getR();
    double dy = cos(resAngle) * this->getR();

    Position pos (this->getCenter().first + dx, this->getCenter().second + dy);

    // We must convert the angle to the tangential angle. This depends on whether
    // this is a left- or right-bend
    if (bendLeft) {
        resAngle += (M_PI/2.0);
        if (resAngle > M_PI)
            resAngle -= (2*M_PI);
    } else {
        resAngle -= (M_PI/2.0);
        if (resAngle < -1*M_PI)
            resAngle += (2*M_PI);
    }

    assert(!std::isnan(pos.first));
    assert(!std::isnan(this->zoom));
    return {pos, resAngle, this->zoom};
}

double CircleTrajectoryItem::getStartAngle()
{
    double dangle = CoordinateUtil::angleDiff(this->getAStart(), this->getAEnd());
    bool bendLeft = (dangle > 0);
    double resAngle = this->getAStart();

    // We must convert the angle to the tangential angle. This depends on whether
    // this is a left- or right-bend
    if (bendLeft) {
        resAngle += (M_PI/2.0);
        if (resAngle > M_PI)
            resAngle -= (2*M_PI);
    } else {
        resAngle -= (M_PI/2.0);
        if (resAngle < -1*M_PI)
            resAngle += (2*M_PI);
    }

    return resAngle;
}

double CircleTrajectoryItem::getEndAngle()
{
    double dangle = CoordinateUtil::angleDiff(this->getAStart(), this->getAEnd());
    bool bendLeft = (dangle > 0);
    double resAngle = this->getAEnd();

    // We must convert the angle to the tangential angle. This depends on whether
    // this is a left- or right-bend
    if (bendLeft) {
        resAngle += (M_PI/2.0);
        if (resAngle > M_PI)
            resAngle -= (2*M_PI);
    } else {
        resAngle -= (M_PI/2.0);
        if (resAngle < -1*M_PI)
            resAngle += (2*M_PI);
    }

    return resAngle;
}

double CircleTrajectoryItem::getLength()
{
    double dangle = CoordinateUtil::angleDiff(this->getAStart(), this->getAEnd());

    Position center = this->getCenter();
    Position point_on_circle(center.first, center.second + this->getR());
    double approximate_radius = Map::compute_distance(center, point_on_circle);

    double fullCircle = 2 * M_PI * approximate_radius;
    double frac = std::abs(dangle / (2 * M_PI));

    return fullCircle * frac;
}

double CircleTrajectoryItem::getStartZoom()
{
    return this->zoom;
}

double CircleTrajectoryItem::getEndZoom()
{
    return this->zoom;
}


CircleTrajectoryItem::CircleTrajectoryItem(Position center, double r, double aStart, double aSpan, double zoom, double speed):
    TrajectoryItem(speed),
    center(center),
    r(r),
    aStart(aStart),
    aEnd(aSpan),
    zoom(zoom)
{
}

int StraightTrajectoryItem::getType() {
    return TrajectoryItem::TYPE_STRAIGHT;
}

Position StraightTrajectoryItem::getStart() const
{
    return start;
}
Position StraightTrajectoryItem::getEnd() const
{
    return end;
}

CarPosition StraightTrajectoryItem::interpolatePosition(double dist)
{
    assert(dist < this->getLength() + DELTA);
    assert(dist >= 0);
    if (dist > this->getLength()) {
        dist = this->getLength();
    }

    double x,y,zoom;

    if (this->getLength() == 0.0) {
      x = this->getStart().first;
      y = this->getStart().second;
      zoom = this->getStartZoom(); // TODO huh... could that jump?
    } else {
      double frac = dist / this->getLength();

      x = (1-frac) * this->getStart().first + frac * this->getEnd().first;
      y = (1-frac) * this->getStart().second + frac * this->getEnd().second;
      zoom = (1-frac) * this->getStartZoom() + frac * this->getEndZoom();
    }

    double angle = this->getAngle();

    assert(!std::isnan(x));
    assert(!std::isnan(zoom));
    return { Position(x,y), angle, zoom };
}

double StraightTrajectoryItem::getLength()
{
  return Map::compute_distance(this->getEnd(), this->getStart());
}

double StraightTrajectoryItem::getAngle()
{
    return CoordinateUtil::getAngle(this->getStart(), this->getEnd());
}

double StraightTrajectoryItem::getStartZoom()
{
    return this->startZoom;
}

double StraightTrajectoryItem::getEndZoom()
{
    return this->endZoom;
}

double StraightTrajectoryItem::getStartAngle()
{
  return this->getAngle();
}

double StraightTrajectoryItem::getEndAngle()
{
  return this->getAngle();
}

void
CircleTrajectoryItem::adjustZoomLevel(double zoom)
{
  this->zoom = zoom;
}

void
StraightTrajectoryItem::adjustZoomLevel(double zoom)
{
  this->startZoom = zoom;
  this->endZoom = zoom;
}

StraightTrajectoryItem::StraightTrajectoryItem(Position start, Position end, double startZoom, double endZoom, double speed):
    TrajectoryItem(speed),
    start(start),
    end(end),
    startZoom(startZoom),
    endZoom(endZoom)
{
  assert(!std::isnan(end.first));
  assert(!std::isnan(end.second));
  assert(!std::isnan(start.first));
  assert(!std::isnan(start.second));
  assert(!std::isnan(startZoom));
  assert(!std::isnan(endZoom));
  assert(endZoom > (-1 * std::numeric_limits<double>::infinity()));
}

TrajectoryItem::~TrajectoryItem()
{
}

int TrajectoryItem::getType()
{
    return TrajectoryItem::TYPE_INVALID;
}

TrajectoryItem::TrajectoryItem(double speed)
  : speed(speed)
{}

double
TrajectoryItem::getSpeed()
{
  return this->speed;
}
