#include "rotatingpoi.h"

#include "util/coordinateutil.h"

#include <QDebug>

#include <cassert>

RotatingPOI::RotatingPOI(Position anchor, Position leftLower, Position rightUpper, POI *poi, double rotation):
    anchor(anchor),
    rotation(rotation),
    poi(poi),
    rotateProj(anchor, rotation, 1.0)
{
  assert(!std::isnan(rotation));
  assert(!std::isnan(anchor.first));
  this->center = Position((leftLower.first + rightUpper.first) / 2.0,
                          (leftLower.second + rightUpper.second) / 2.0);
  this->height = (rightUpper.second - leftLower.second);
  this->width = (rightUpper.first - leftLower.first);
  this->anchorDist = this->center.second - (this->height / 2.0) - this->anchor.second;

  assert(!std::isnan(this->center.first));
  //assert(this->getCorner(true, true).second > this->center.second);
  /*
  std::cout << "This is a new RPOI. Unrotated: \n";
  this->rotation = 0.0;
  this->dbgPrintToWolfram();
  std::cout << "This is a new RPOI. Rotated 45 degrees: \n";
  this->rotation = M_PI / 4.0;
  this->dbgPrintToWolfram();
  */
  this->rotation = rotation;
}

bool RotatingPOI::aLeftOfT()
{
    Position g1 = Position(this->center.first - (this->width / 2.0), this->center.second + (this->height / 2.0));
    Position g2 = Position(this->center.first + (this->width / 2.0), this->center.second + (this->height / 2.0));

    return CoordinateUtil::xLeftOfG(this->anchor, g1, g2);
}

bool RotatingPOI::aLeftOfB()
{
    Position g1 = Position(this->center.first - (this->width / 2.0), this->center.second - (this->height / 2.0));
    Position g2 = Position(this->center.first + (this->width / 2.0), this->center.second - (this->height / 2.0));

    return CoordinateUtil::xLeftOfG(this->anchor, g1, g2);
}

bool RotatingPOI::aLeftOfL()
{
    Position g1 = Position(this->center.first - (this->width / 2.0), this->center.second + (this->height / 2.0));
    Position g2 = Position(this->center.first - (this->width / 2.0), this->center.second - (this->height / 2.0));

    return CoordinateUtil::xLeftOfG(this->anchor, g1, g2);
}

bool RotatingPOI::aLeftOfR()
{
    Position g1 = Position(this->center.first + (this->width / 2.0), this->center.second + (this->height / 2.0));
    Position g2 = Position(this->center.first + (this->width / 2.0), this->center.second - (this->height / 2.0));

    return CoordinateUtil::xLeftOfG(this->anchor, g1, g2);
}
Position RotatingPOI::getAnchor() const
{
    return anchor;
}
Position RotatingPOI::getCenter() const
{
    double x = this->center.first;
    double y = this->center.second;
    this->rotateProj.project(&x, &y);

    // Revert the "center on anchor" done by the projection
    y += this->anchor.second;
    x += this->anchor.first;

    return Position(x,y);
}

Position RotatingPOI::getCorner(bool top, bool left) const
{
    double x = this->center.first;
    if (left) {
        x -= this->width / 2.0;
    } else {
        x += this->width / 2.0;
    }

    double y = this->center.second;
    if (top) {
        y += this->height / 2.0;
    } else {
        y -= this->height / 2.0;
    }

    assert(!std::isnan(x));
    this->rotateProj.project(&x, &y);
    assert(!std::isnan(x));

    // Revert the "center on anchor" done by the projection
    y += this->anchor.second;
    x += this->anchor.first;

    return Position(x, y);
}

void RotatingPOI::dbgPrintToWolfram() const {
  assert(!std::isnan(this->getCorner(true, true).first));
  //std::cout << "Width: " << this->getWidth() << " Height: " << this->getHeight() << "\n";
  std::cout << std::fixed << std::setprecision(5) << "{{{" << this->getCorner(true, true).first << "," << this->getCorner(true, true).second << "},{" << this->getCorner(true, false).first << "," << this->getCorner(true, false).second << "},{" << this->getCorner(false, false).first << "," << this->getCorner(false, false).second << "},{"  << this->getCorner(false, true).first << "," << this->getCorner(false, true).second << "}},{{" << this->getAnchor().first << "," << this->getAnchor().second << "},{" << this->getCenter().first << "," << this->getCenter().second << "}}}";
}

double RotatingPOI::getHeight() const
{
    return height;
}
double RotatingPOI::getWidth() const
{
    return width;
}

double RotatingPOI::getBaseDist() const
{
    return this->anchorDist;
}

POI *RotatingPOI::getPoi() const
{
    return poi;
}
double RotatingPOI::getRotation() const
{
    return rotation;
}

void RotatingPOI::setRotation(double value)
{
    assert(!std::isnan(value));
    rotation = value;
    this->rotateProj = CameraProjection(this->getAnchor(), value, 1.0);
}
