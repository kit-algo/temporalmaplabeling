#include "projection.h"

#include "proj_api.h"
#include "float.h"

MercatorProjection::MercatorProjection()
{
    this->from = pj_init_plus("+proj=latlong +ellps=WGS84 +no_defs");
    this->to = pj_init_plus("+proj=merc +ellps=clrk66 +lat_ts=33");
}

MercatorProjection::~MercatorProjection()
{
    pj_free(this->from);
    pj_free(this->to);
}

void MercatorProjection::project(double *x, double *y) const
{
    (*x) *= DEG_TO_RAD;
    (*y) *= DEG_TO_RAD;

    pj_transform(this->from, this->to, 1, 1, x, y, NULL );
}

void MercatorProjection::reverseProject(double *x, double *y) const
{
    pj_transform(this->to, this->from, 1, 1, x, y, NULL );

    (*x) /= DEG_TO_RAD;
    (*y) /= DEG_TO_RAD;
}


ScreenCoordProjection::ScreenCoordProjection(std::set<Position> positions, double maxX, double maxY):
    screenMaxX(maxX), screenMaxY(maxY)
{
    this->determineInputBounds(positions);
}

void ScreenCoordProjection::project(double *x, double *y) const
{
    double spanX = this->inputMaxX - this->inputMinX;
    double spanY = this->inputMaxY - this->inputMinY;
    double dx = *x - this->inputMinX;
    double dy = *y - this->inputMinY;

    double projectedX = (dx / spanX) * this->screenMaxX;
    double projectedY = (dy / spanY) * this->screenMaxY;

    *x = projectedX;
    *y = projectedY;
}

void ScreenCoordProjection::determineInputBounds(std::set<Position> positions)
{
    this->inputMinX = DBL_MAX;
    this->inputMaxX = -DBL_MAX;
    this->inputMaxY = -DBL_MAX;
    this->inputMinY = DBL_MAX;

    for (auto waypoint : positions) {
        this->inputMaxX = std::max(waypoint.first, this->inputMaxX);
        this->inputMinX = std::min(waypoint.first, this->inputMinX);
        this->inputMaxY = std::max(waypoint.second, this->inputMaxY);
        this->inputMinY = std::min(waypoint.second, this->inputMinY);
    }

}


CameraProjection::CameraProjection(Position center, double rotation, double zoom):
    center(center),
    rotation(rotation),
    zoom(zoom)
{
}

void
CameraProjection::project(double *x, double *y) const
{
    double x_tmp = *x;
    double y_tmp = *y;

    // Translate
    x_tmp -= this->center.first;
    y_tmp -= this->center.second;

    // Stretch
    x_tmp *= this->zoom;
    y_tmp *= this->zoom;

    // Rotate
    *x = cos(this->rotation) * x_tmp - sin(this->rotation) * y_tmp;
    *y = sin(this->rotation) * x_tmp + cos(this->rotation) * y_tmp;
}

double CameraProjection::reverseProjectDistance(double dist) const
{
    return dist / this->zoom;
}

double CameraProjection::projectDistance(double dist) const
{
    return dist * this->zoom;
}


void CameraProjection::reverseProject(double *x, double *y) const
{
    double x_tmp = *x;
    double y_tmp = *y;

    // Rotate back
    *x = cos(-1 * this->rotation) * x_tmp - sin(-1 * this->rotation) * y_tmp;
    *y = sin(-1 * this->rotation) * x_tmp + cos(-1 * this->rotation) * y_tmp;

    // Unstretch
    *x /= this->zoom;
    *y /= this->zoom;

    // Translate
    *x += this->center.first;
    *y += this->center.second;
}
