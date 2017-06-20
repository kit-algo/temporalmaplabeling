#ifndef ROTATINGPOIFACTORY_H
#define ROTATINGPOIFACTORY_H

#include "map/projection.h"
#include "map/map.h"
#include "rotatingpoi.h"

#include <QFont>
#include <QFontMetricsF>

class RotatingPOIFactory
{
public:
    RotatingPOIFactory(double width, double height, QFont font, CameraProjection proj);

    RotatingPOI convert(POI *poi);

private:
    CameraProjection proj;
    QFont font;
    QFontMetricsF fontMetrics;
    double camera_width;
    double camera_height;

    static std::map<std::string, std::pair<double, double>> bbox_cache;
};

#endif // ROTATINGPOIFACTORY_H
