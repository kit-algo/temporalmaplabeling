#include "rotatingpoifactory.h"

#include <QRect>
#include <QString>
#include <QDebug>

#include "config.h"

std::map<std::string, std::pair<double, double>> RotatingPOIFactory::bbox_cache;

RotatingPOIFactory::RotatingPOIFactory(double width, double height, QFont font, CameraProjection proj):
    proj(proj),
    font(font),
    fontMetrics(font),
    camera_width(width),
    camera_height(height)
{
}

RotatingPOI RotatingPOIFactory::convert(POI *poi)
{
    Position anchor = poi->getPos();

    double width;
    double height;

    if (RotatingPOIFactory::bbox_cache.find(poi->getLabel()) != RotatingPOIFactory::bbox_cache.end()) {
        std::pair<double, double> dimensions = RotatingPOIFactory::bbox_cache[poi->getLabel()];
        width = dimensions.first;
        height = dimensions.second;
    } else {
        QRectF bbox = this->fontMetrics.boundingRect(QString::fromStdString(poi->getLabel()));
        height = bbox.height();
        width = bbox.width();
        RotatingPOIFactory::bbox_cache.insert(std::make_pair(poi->getLabel(), std::make_pair(width, height)));
    }

    //std::cout << "Converting POI of box dimensions " << width << "x" << height << "\n";

    width = this->proj.reverseProjectDistance(width);
    height = this->proj.reverseProjectDistance(height);

    //std::cout << "After projection: " << width << "x" << height << "\n";

    double anchorDist = this->camera_height * ANCHOR_DIST_PERCENT / 100.0;
    anchorDist = this->proj.reverseProjectDistance(anchorDist);

    anchorDist += height / 2.0;

    double centerX = anchor.first;
    assert(!std::isnan(centerX));
    double centerY = anchor.second + anchorDist;
    assert(!std::isnan(centerY));

    Position leftLower (centerX - width / 2.0, centerY - height / 2.0);
    Position rightUpper (centerX + width / 2.0, centerY + height / 2.0);

    return RotatingPOI(anchor, leftLower, rightUpper, poi, 0.0);
}
