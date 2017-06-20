#ifndef CONFLICT_H
#define CONFLICT_H

#include "map/map.h"

class Conflict
{
public:
    Conflict(double angleStart, double angleEnd, double minZoom, POI *poi1, POI *poi2);

    /* BEWARE: The following *might* be set to NaN (check with "std::isnan()")
     * to indicate that a conflict is not happening.
     */
    double getAngleEnd() const;
    double getAngleStart() const;
    double getMinZoom() const;
    POI *getPoi1() const;
    POI *getPoi2() const;

private:
    double angleStart;
    double angleEnd;
    double minZoom;
    POI *poi1;
    POI *poi2;
};

struct ConflictEvent {
    POI *poi1;
    POI *poi2;
    double dist;
    bool start;
    int id;

    static int next_id;
};

struct VisibilityEvent {
    POI *poi;
    double dist;
    bool start;
};

#endif // CONFLICT_H
