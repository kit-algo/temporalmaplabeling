#include "camera.h"

#include "rotatingpoifactory.h"
#include "conflict.h"
#include "config.h"
#include "util/coordinateutil.h"
#include "util/debugging.h"

#include <functional>
#include <algorithm>

#include <QDebug>

#include "util/clock.h"

#include <boost/range/adaptor/reversed.hpp>

Camera::Camera(double width, double height, Trajectory *trajectory, Map *map, QFont font):
    width(width),
    height(height),
    map(map),
    trajectory(trajectory),
    font(font)
{
    this->computeMaxLabelSpan();
}

ConflictIntervals &
Camera::getConflictIntervals()
{
  return this->conflicts;
}

ConflictIntervals &
Camera::getDisplayPairs()
{
  return this->displayPairs;
}

VisibilityIntervals &
Camera::getVisibilityIntervals()
{
  return this->visibilityIntervals;
}

void Camera::compute()
{
    std::cout << "Making Display Pairs..." << "\n";
    Clock interpolate_clock;
    interpolate_clock.start();

    TrajectoryFilter tf(this->trajectory, this, this->map);

    this->displayPairs = tf.getDisplayPairs();
    this->visibilityIntervals = tf.getVisibilityIntervals();

    /* The conflicts are taken directly from the TrajectoryFilter
     * by sampling!
     */
    this->conflicts = tf.getConflicts();

}

std::vector<RotatingPOI> Camera::getPOIsForCar(CarPosition carpos, bool onlyVisible)
{
    std::vector<RotatingPOI> res;

    CameraProjection cp = this->getProjectionForCar(carpos);
    RotatingPOIFactory fac(this->width, this->height, this->font, cp);

    if (! onlyVisible) {
        for (auto poi : this->map->getPOI()) {
          //std::cout << "**** gPFC: " << (poi->getLabel()) << "(" << (poi->getId()) << ")" << "\n";
          RotatingPOI rpoi = fac.convert(poi);
          rpoi.setRotation(CoordinateUtil::mapAngleToRPOI(carpos.angle));
          res.push_back(rpoi);
        }
    } else {
        CameraProjection proj(carpos.pos, carpos.angle, carpos.zoom);
        double width = proj.reverseProjectDistance(this->getWidth());
        double height = proj.reverseProjectDistance(this->getHeight());
        width += proj.reverseProjectDistance(this->maxLabelSpan);
        height += proj.reverseProjectDistance(this->maxLabelSpan);
        width *= OVERSCAN_FACTOR; // For good measure
        height *= OVERSCAN_FACTOR;

        Position leftLower = { -1 * width / 2.0, -1 * height / 2.0};
        Position rightUpper = { width / 2.0, height / 2.0};
        leftLower.first += carpos.pos.first;
        leftLower.second += carpos.pos.second;
        rightUpper.first += carpos.pos.first;
        rightUpper.second += carpos.pos.second;

        for (auto poi : this->map->getPOIsWithin(leftLower, rightUpper)) {
            RotatingPOI rpoi = fac.convert(poi);
            rpoi.setRotation(CoordinateUtil::mapAngleToRPOI(carpos.angle));
            res.push_back(rpoi);
        }
    }

    return res;
}

CameraProjection Camera::getProjectionForCar(CarPosition carpos)
{
    return CameraProjection(carpos.pos, carpos.angle, carpos.zoom);
}
double Camera::getWidth() const
{
    return width;
}
double Camera::getHeight() const
{
    return height;
}

std::vector<InterpolationStep> Camera::getInterpolationPoints()
{
    double dist = 0.0;
    std::vector<double> points;
    std::vector<InterpolationStep> res;

    while (dist < this->trajectory->getLength()) {
        points.push_back(dist);
        dist += INTERPOLATION_STEP;
    };
    points.push_back(this->trajectory->getLength());

    std::vector<CarPosition> carpos = this->trajectory->interpolatePositions(points);

    for (size_t i = 0 ; i < points.size() ; i++) {
      InterpolationStep step = {this->getViewportAt(carpos[i]), carpos[i], points[i]};
      res.push_back(step);
    }

    return res;
}

void Camera::precomputeRequirements()
{
    if (this->trajectory == nullptr)
        return;

    double dist = 0.0;
    double zoomStartedAt = 0.0;
    double zoomEndedAt = 0.0;
    double currentZoom = this->trajectory->getItems()[0]->getStartZoom();
    //double angleStartedAt = 0.0;
    //double angleEndedAt = 0.0;
    //double currentAngle = this->trajectory->getItems()[0]->getStartAngle();

    for (auto item : this->trajectory->getItems()) {
        double endDist = dist + item->getLength();

        if (item->getStartAngle() != item->getEndAngle()) {
            assert(item->getType() == TrajectoryItem::TYPE_CIRCLE);

            if (item->getStartZoom() != currentZoom) {
              assert(currentZoom != 1.0);
              this->zoomLevels.insert(std::make_tuple(currentZoom, zoomStartedAt, zoomEndedAt));
              currentZoom = item->getStartZoom();
              zoomStartedAt = dist;

            }
            zoomEndedAt = endDist;
        }

        if (item->getStartZoom() != item->getEndZoom()) {
            assert(item->getType() == TrajectoryItem::TYPE_STRAIGHT);
            StraightTrajectoryItem *straight = dynamic_cast<StraightTrajectoryItem *>(item);

            this->zoomAngles.insert(std::make_tuple(straight->getAngle(), dist, endDist));
        }
        dist = endDist;
    }

    this->zoomLevels.insert(std::make_tuple(currentZoom, zoomStartedAt, zoomEndedAt));
}

void Camera::computeMaxLabelSpan()
{
    double maxSpan = std::numeric_limits<double>::min();

    for (auto poi : this->getPOIsForCar({Position(0.0, 0.0), 0.0, 1.0})) {
        Position anchor = poi.getAnchor();
        Position leftUpper = poi.getCorner(true, true);
        double dx = anchor.first - leftUpper.first;
        double dy = anchor.second - leftUpper.second;

        double span = std::sqrt(dx * dx + dy * dy);
        maxSpan = std::max(maxSpan, span);
    }

    this->maxLabelSpan = maxSpan;
}

RotatingPOI Camera::getViewportAt(CarPosition pos)
{
  if (this->trajectory == nullptr) {
      return RotatingPOI(Position(0,0), { -1 * width / 2.0, -1 * height / 2.0}, { width / 2.0, height / 2.0}, nullptr, CoordinateUtil::mapAngleToRPOI(0));
  }
  CameraProjection proj(pos.pos, pos.angle, pos.zoom);

  double width = proj.reverseProjectDistance(this->getWidth());
  double height = proj.reverseProjectDistance(this->getHeight());

  Position leftLower = { -1 * width / 2.0, -1 * height / 2.0};
  Position rightUpper = { width / 2.0, height / 2.0};
  Position center = { 0.0, 0.0 };

  leftLower.first += pos.pos.first;
  rightUpper.first += pos.pos.first;
  center.first += pos.pos.first;

  leftLower.second += pos.pos.second;
  rightUpper.second += pos.pos.second;
  center.second += pos.pos.second;

  //proj.reverseProject(&(leftLower.first), &(leftLower.second));
  //proj.reverseProject(&(rightUpper.first), &(rightUpper.second));
  //proj.reverseProject(&(center.first), &(center.second));

  //std::cout << "Returning Viewport at angle " << pos.angle << "\n";
  RotatingPOI res(center, leftLower, rightUpper, nullptr, CoordinateUtil::mapAngleToRPOI(pos.angle));
  //std::cout << "Camera dimensions: " << this->getWidth() << " x " << this->getHeight() << "\n";
  //std::cout << "Viewport dimensions: " << res.getWidth() << " x " << res.getHeight() << "\n";
  return res;
}

RotatingPOI Camera::getViewportAt(double dist)
{
    if (this->trajectory == nullptr) {
        return RotatingPOI(Position(0,0), { -1 * width / 2.0, -1 * height / 2.0}, { width / 2.0, height / 2.0}, nullptr, CoordinateUtil::mapAngleToRPOI(0));
    }
    CarPosition pos = this->trajectory->interpolatePosition(dist);
    return this->getViewportAt(pos);
}

std::set<std::set<POI *> > Camera::getConflictsAt(double dist)
{
  Interval lookup = make_interval(dist, dist);
  std::set<std::set<POI *>> res;
  res = this->conflicts.query(bgi::intersects(lookup));
  return res;
  /*
    Interval lookup = boost::icl::construct<Interval>(dist, dist, boost::icl::interval_bounds::closed());
    assert(! boost::icl::is_empty(lookup));
    auto conflictIt = this->conflicts.find(lookup);
    if (conflictIt == this->conflicts.end())
        return {{}};
    return conflictIt->second;
    */
}

std::set<POI *> Camera::getVisibleAt(double dist)
{
    //Interval lookup = boost::icl::construct<Interval>(dist, dist, boost::icl::interval_bounds::closed());
    Interval lookup = make_interval(dist, dist);
    std::set<POI *> res;
    res = this->visibilityIntervals.query(bgi::intersects(lookup));
    return res;
}
