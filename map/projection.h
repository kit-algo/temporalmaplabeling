#ifndef PROJECTION_H
#define PROJECTION_H

#include "proj_api.h"
#include "position.h"

#include <exception>
#include <set>

class Map;

class Projection
{
public:
    virtual void project(double *x, double *y) const = 0;
    virtual void reverseProject(double *x, double *y) const = 0;

    // Only possible in angle-conform projections
    virtual double projectAngle(double angle) const = 0;
    virtual double reverseProjectAngle(double angle) const = 0;

    // Only possible in distance-conform projections
    virtual double projectDistance(double dist) const = 0;
    virtual double reverseProjectDistance(double dist) const = 0;

    virtual ~Projection() {};
};

class MercatorProjection : Projection
{
public:
    MercatorProjection();
    ~MercatorProjection();

    void project(double *x, double *y) const;
    void reverseProject(double *x, double *y) const;

    // Not implemented here
    double projectAngle(double angle) const { (void)angle; throw "MP, PA"; }
    double reverseProjectAngle(double angle) const { (void)angle; throw "MP, rPA"; }
    double projectDistance(double dist) const { (void)dist; throw "MP, pD"; }
    double reverseProjectDistance(double dist) const { (void)dist; throw "MP, rPD"; }

private:
    projPJ from;
    projPJ to;
};

class ScreenCoordProjection : Projection
{
public:
    ScreenCoordProjection(std::set<Position> positions, double maxX, double maxY);

    void project(double *x, double *y) const;

    // Not implemented here
    void reverseProject(double *x, double *y) const { (void)x; (void)y; throw "SCP, rP"; }
    double projectAngle(double angle) const { (void)angle; throw "SCP, PA"; }
    double reverseProjectAngle(double angle) const { (void)angle; throw "SCP, rPA"; }
    double projectDistance(double dist) const { (void)dist; throw "SCP, PD"; }
    double reverseProjectDistance(double dist) const { (void)dist; throw "SCP, rPD"; }

private:
    void determineInputBounds(std::set<Position> positions);

    Map *map;
    double screenMaxX;
    double screenMaxY;
    double inputMaxX;
    double inputMaxY;
    double inputMinX;
    double inputMinY;
};

class CameraProjection : Projection
{
public:
    CameraProjection(Position center, double rotation, double zoom);

    void project(double *x, double *y) const;

    // Not implemented here
    double projectAngle(double angle) const { (void)angle; throw "CP, PA"; }
    double reverseProjectAngle(double angle) const { (void)angle; throw "CP, rPA"; }

    double projectDistance(double dist) const;
    double reverseProjectDistance(double dist) const;
    void reverseProject(double *x, double *y) const;
private:
    Position center;
    double rotation;
    double zoom;
};

#endif // PROJECTION_H
