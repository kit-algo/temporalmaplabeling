#ifndef ROTATINGPOI_H
#define ROTATINGPOI_H

#include "map/map.h"
#include "map/projection.h"

#include <QRectF>

class RotatingPOI
{
public:
    RotatingPOI(Position anchor, Position leftLower, Position rightUpper, POI *poi, double rotation);

    bool aLeftOfT();
    bool aLeftOfB();
    bool aLeftOfL();
    bool aLeftOfR();

    Position getAnchor() const;
    Position getCenter() const;
    Position getCorner(bool top, bool left) const;
    double getHeight() const;
    double getWidth() const;
    double getBaseDist() const;

    POI *getPoi() const;

    double getRotation() const;
    void setRotation(double value);

    void dbgPrintToWolfram() const;

private:
    Position anchor;
    Position center;
    double height;
    double width;
    double rotation;
    double anchorDist;

    POI *poi;
    CameraProjection rotateProj;
};

#endif // ROTATINGPOI_H
