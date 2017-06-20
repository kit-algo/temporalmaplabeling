#include "conflict.h"

Conflict::Conflict(double angleStart, double angleEnd, double minZoom, POI *poi1, POI *poi2):
    angleStart(angleStart),
    angleEnd(angleEnd),
    minZoom(minZoom),
    poi1(poi1),
    poi2(poi2)
{

}
double Conflict::getAngleEnd() const
{
    return angleEnd;
}
double Conflict::getAngleStart() const
{
    return angleStart;
}
double Conflict::getMinZoom() const
{
    return minZoom;
}
POI *Conflict::getPoi2() const
{
    return poi2;
}



POI *Conflict::getPoi1() const
{
    return poi1;
}
