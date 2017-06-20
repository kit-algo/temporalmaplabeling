#ifndef COORDINATEUTIL_H
#define COORDINATEUTIL_H

#include "map/map.h"
#include "algvec.h"

#include "math.h"
#include "proj_api.h"
#include "conflicts/rotatingpoi.h"

#include <QDebug>
#include <armadillo>

#include "clipper.h"

#define CGAL_NDEBUG

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 CGALPoint;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

using namespace ClipperLib;

class CoordinateUtil
{
public:
    CoordinateUtil();

    static Position interpolate(Position pos1, Position pos2, double length) {
        AlgVec base = AlgVec(pos1.first, pos1.second);
        AlgVec direction = AlgVec(pos2.first - pos1.first, pos2.second - pos1.second);
        direction = direction.normalize();

        AlgVec res = base + (direction * length);
        return Position(res.x, res.y);
    }

    /* Returns the angle pointing from pos1 to pos2, with 0 being "up", and being counted
     * counterclockwise. */
    static double getAngle(Position pos1, Position pos2) {
        //return std::atan2(pos2.second - pos1.second, pos2.first - pos1.first);;
        double angle = std::atan2(pos2.second - pos1.second, pos2.first - pos1.first);

        angle -= (M_PI / 2.0);
        if (angle < -1 * (M_PI)) {
            angle += 2*M_PI;
        }

        return angle;
    }

    static double mapAngleToRPOI(double angle) {
      return angle;
    }

    static double RPOIAngleToMap(double angle) {
        return angle;
    }

    /* Angle is assumed to be between -Pi and Pi */
    static int radToQt(double angle) {
        /* Our angle is between [-pi; pi] and has ccw orientation
         * and has 0° pointing up
         *
         * Qt uses [0; 360] and cw orientation.
         * and has 0° pointing right
        */

        // Our coordinate system has 0° pointing up
        // Qt has (sanely..) 0° pointing right
        angle += M_PI / 2.0;

        angle *= -1; // convert to cw, now [0, 2pi]
        //angle += M_PI; // shift to [0; -2pi]
        angle *= RAD_TO_DEG; // shift to [0; 360]

        return angle;
    }

    static double dist(Position from, Position to) {
        double dx = from.first - to.first;
        double dy = from.second - to.second;
        return sqrt(dx*dx + dy*dy);
    }

    static bool xLeftOfG(Position x, Position g1, Position g2) {
        arma::vec normal ({g1.second - g2.second, g2.first - g1.first});
        arma::vec toX ({x.first - g1.first, x.second - g1.second});

        return arma::dot(normal, toX) > 0;
    }

    static double distLinePoint(Position point, Position g1, Position g2) {
        // We have to go to 3 dimensions to make the cross product work
        // I'm too lazy to implement this myself for two dimensions.
        arma::vec line ({g2.first - g1.first, g2.second - g1.second, 0.0});
        arma::vec toPoint ({point.first - g1.first, point.second - g1.second, 0.0});

        double area = arma::norm(arma::cross(line, toPoint), 2); // Area of the parallelogram...
        double height = area / arma::norm(line, 2);
        return abs(height);
    }

    static double angleDiff(double angleStart, double angleEnd) {
        double dangle = angleEnd - angleStart;
        dangle += M_PI;
        dangle = fmod(((fmod(dangle, (2*M_PI))) + 2*M_PI), (2*M_PI));
        dangle -= M_PI;

        return dangle;
    }

#define SLOPE_DELTA 0.1
  static bool rpoiOverlapHorizontal(RotatingPOI *rpoiBottom, RotatingPOI *rpoiTop) {
/*    osmium::object_id_type id1 = rpoiBottom->getPoi()->getId();
    osmium::object_id_type id2 = rpoiTop->getPoi()->getId();
    bool dbg_enable = false;
    if (((id1 == 524555515) && (id2 == 3506002970)) ||
      ((id1 == 3506002970) && (id2 == 524555515))) {
        dbg_enable = true;
        std::cout << "~~~~~ DBG ENABLE HORIZONTAL\n";
    }
*/

    Position tLU = rpoiTop->getCorner(true, true);
    //Position tLL = rpoiTop->getCorner(false, true);
    Position tRU = rpoiTop->getCorner(true, false);
    //Position tRL = rpoiTop->getCorner(false, false);

    //Position bLU = rpoiBottom->getCorner(true, true);
    Position bLL = rpoiBottom->getCorner(false, true);
    //Position bRU = rpoiBottom->getCorner(true, false);
    Position bRL = rpoiBottom->getCorner(false, false);

    double p_slope = rpoiTop->getRotation();

    double bottomMin, bottomMax, topMin, topMax;
    if (((((M_PI/2.0) - SLOPE_DELTA) < std::abs(p_slope)) && (((M_PI/2.0) + SLOPE_DELTA) > std::abs(p_slope))) ||
        (((-1*(M_PI/2.0) - SLOPE_DELTA) < std::abs(p_slope)) && ((-1*(M_PI/2.0) + SLOPE_DELTA) > std::abs(p_slope)))) {
      // Use y axis
      bottomMin = std::min(bLL.second, bRL.second);
      bottomMax = std::max(bLL.second, bRL.second);
      topMin = std::min(tLU.second, tRU.second);
      topMax = std::max(tLU.second, tRU.second);
    } else {
      // Use x axis
      bottomMin = std::min(bLL.first, bRL.first);
      bottomMax = std::max(bLL.first, bRL.first);
      topMin = std::min(tLU.first, tRU.first);
      topMax = std::max(tLU.first, tRU.first);
    }

    return ((topMin <= bottomMax) && (topMax >= bottomMin));
  }

  static bool rpoiOverlapVertical(RotatingPOI *rpoiLeft, RotatingPOI *rpoiRight) {
    osmium::object_id_type id1 = rpoiLeft->getPoi()->getId();
    osmium::object_id_type id2 = rpoiLeft->getPoi()->getId();
    bool dbg_enable = false;
    if (((id1 == 524555515) && (id2 == 3506002970)) ||
      ((id1 == 3506002970) && (id2 == 524555515))) {
        dbg_enable = true;
        std::cout << "~~~~~ DBG ENABLE\n";
    }


    //Position rLU = rpoiRight->getCorner(true, true);
    //Position rLL = rpoiRight->getCorner(false, true);
    Position rRU = rpoiRight->getCorner(true, false);
    Position rRL = rpoiRight->getCorner(false, false);

    Position lLU = rpoiLeft->getCorner(true, true);
    Position lLL = rpoiLeft->getCorner(false, true);
    //Position lRU = rpoiLeft->getCorner(true, false);
    //Position lRL = rpoiLeft->getCorner(false, false);

    double p_slope = rpoiLeft->getRotation();

    if (dbg_enable) {
      std::cout << "Slope is: " << p_slope << "\n";
    }

    double leftMin, leftMax, rightMin, rightMax;
    //if ((std::abs((M_PI/2.0) - p_slope) <= SLOPE_DELTA) || (std::abs((M_PI/-2.0) - p_slope) <= SLOPE_DELTA)) {
    if (((((M_PI/2.0) - SLOPE_DELTA) < std::abs(p_slope)) && (((M_PI/2.0) + SLOPE_DELTA) > std::abs(p_slope))) ||
        (((-1*(M_PI/2.0) - SLOPE_DELTA) < std::abs(p_slope)) && ((-1*(M_PI/2.0) + SLOPE_DELTA) > std::abs(p_slope)))) {
      // Use x axis

      if (dbg_enable) {
        std::cout << "X Axis\n";
      }
      leftMin = std::min(lLU.first, lLL.first);
      leftMax = std::min(lLU.first, lLL.first);
      rightMin = std::min(rRU.first, rRL.first);
      rightMax = std::max(rRU.first, rRL.first);
    } else {
      // Use y axis

      if (dbg_enable) {
        std::cout << "Y Axis\n";
      }
      leftMin = std::min(lLU.second, lLL.second);
      leftMax = std::min(lLU.second, lLL.second);
      rightMin = std::min(rRU.second, rRL.second);
      rightMax = std::max(rRU.second, rRL.second);
    }

    if (dbg_enable) {
      std::cout << leftMin << "->" << leftMax << " / " << rightMin << "->" << rightMax << "\n";
    }

    return ((leftMin <= rightMax) && (leftMax >= rightMin));
  }

  static Position rotatePos(Position pos, double angle) {
    double new_x = pos.first * std::cos(angle) - pos.second * std::sin(angle);
    double new_y = pos.first * std::sin(angle) + pos.second * std::cos(angle);

    return Position(new_x, new_y);
  }

  static bool alignedRpoiOverlap(RotatingPOI *rpoi1, RotatingPOI *rpoi2) {
    Position aLU = rpoi1->getCorner(true, true);
    Position aLL = rpoi1->getCorner(false, true);
    //Position aRU = rpoi1->getCorner(true, false);
    Position aRL = rpoi1->getCorner(false, false);
    Position bLU = rpoi2->getCorner(true, true);
    Position bLL = rpoi2->getCorner(false, true);
    //Position bRU = rpoi2->getCorner(true, false);
    Position bRL = rpoi2->getCorner(false, false);

    if ((aLU.first != aLL.first) || (aLL.second != aRL.second) || (bLU.first != bLL.first) || (bLL.second != bRL.second)) {
      throw "This may only be used with axis-aligned RPOIs!";
    }

    if (aLU.second < aLL.second) {
      throw "Please rotate upwards.";
    }

    bool overlap_horizontal = ((aLL.first <= bRL.first) && (aRL.first >= bLL.first));
    bool overlap_vertical = ((aLL.second <= bLU.second) && (aLU.second >= bLL.second));

    return overlap_vertical && overlap_horizontal;
  }

  static bool rpoiOverlap(const RotatingPOI *rpoi1, const RotatingPOI *rpoi2) {
    Position aLU = rpoi1->getCorner(true, true);
    Position aLL = rpoi1->getCorner(false, true);
    Position aRU = rpoi1->getCorner(true, false);
    Position aRL = rpoi1->getCorner(false, false);
    Position bLU = rpoi2->getCorner(true, true);
    Position bLL = rpoi2->getCorner(false, true);
    Position bRU = rpoi2->getCorner(true, false);
    Position bRL = rpoi2->getCorner(false, false);

    CGALPoint aPolyPoints[] = { posToCGAL(aLU), posToCGAL(aLL), posToCGAL(aRL), posToCGAL(aRU) };
    CGALPoint bPolyPoints[] = { posToCGAL(bLU), posToCGAL(bLL), posToCGAL(bRL), posToCGAL(bRU) };

    Polygon_2 aPoly = Polygon_2(aPolyPoints, aPolyPoints+4);
    Polygon_2 bPoly = Polygon_2(bPolyPoints, bPolyPoints+4);

    bool result = CGAL::do_intersect(aPoly, bPoly);

    return result;
  }


    static double rpoiOverlapArea(const RotatingPOI *rpoi1, const RotatingPOI *rpoi2) {
      Position aLU = rpoi1->getCorner(true, true);
      Position aLL = rpoi1->getCorner(false, true);
      Position aRU = rpoi1->getCorner(true, false);
      Position aRL = rpoi1->getCorner(false, false);
      Position bLU = rpoi2->getCorner(true, true);
      Position bLL = rpoi2->getCorner(false, true);
      Position bRU = rpoi2->getCorner(true, false);
      Position bRL = rpoi2->getCorner(false, false);

      /*
      CGALPoint aPolyPoints[] = { posToCGAL(aLU), posToCGAL(aLL), posToCGAL(aRL), posToCGAL(aRU) };
      CGALPoint bPolyPoints[] = { posToCGAL(bLU), posToCGAL(bLL), posToCGAL(bRL), posToCGAL(bRU) };

      Polygon_2 aPoly = Polygon_2(aPolyPoints, aPolyPoints+4);
      Polygon_2 bPoly = Polygon_2(bPolyPoints, bPolyPoints+4);

      std::list<Polygon_with_holes_2> intersections;

      CGAL::intersection(aPoly, bPoly, std::back_inserter(intersections));

      bool resultingArea = 0.0;
      for (auto poly : intersections) {
        resultingArea += std::abs(CGAL::to_double(poly.outer_boundary().area()));
      }

      //return resultingArea;
      //bool result = CGAL::do_intersect(aPoly, bPoly);

      //return result;
      */
      Paths poly1(1), poly2(1), solution;

      poly1[0] << posToClipper(aLU) << posToClipper(aLL) << posToClipper(aRL) << posToClipper(aRU);
      poly2[0] << posToClipper(bLU) << posToClipper(bLL) << posToClipper(bRL) << posToClipper(bRU);

      Clipper c;
	    c.AddPaths(poly1, ptSubject, true);
	    c.AddPaths(poly2, ptClip, true);
	    c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);

      assert(solution.size() <= 1);

      unsigned long long clipperArea;
      if (solution.size() == 0 ) {
        clipperArea = 0;
      } else {
        clipperArea = Area(solution[0]) / (100000l * 100000l);
        //std::cout << "Clipper vs. CGAL: " << resultingArea << " vs " << clipperArea << "\n";
      }
      return clipperArea;
    }

/*
#define SLOPE_DELTA 0.05
    static bool rpoiOverlap(RotatingPOI *rpoi1, RotatingPOI *rpoi2) {
        Position aLU = rpoi1->getCorner(true, true);
        Position aLL = rpoi1->getCorner(false, true);
        Position aRU = rpoi1->getCorner(true, false);
        Position aRL = rpoi1->getCorner(false, false);
        Position bLU = rpoi2->getCorner(true, true);
        Position bLL = rpoi2->getCorner(false, true);
        Position bRU = rpoi2->getCorner(true, false);
        Position bRL = rpoi2->getCorner(false, false);

        double p_slope = CoordinateUtil::getAngle(rpoi1->getAnchor(), rpoi2->getAnchor());

        double poi1Min, poi1Max, poi2Min, poi2Max;
        if (((M_PI - SLOPE_DELTA) < std::abs(p_slope)) && ((M_PI + SLOPE_DELTA) > std::abs(p_slope))) {
          // Use y axis
          poi1Min = std::min(aLU.second, aLL.second);
          poi1Min = std::min(poi1Min, aRU.second);
          poi1Min = std::min(poi1Min, aRL.second);

          poi1Max = std::max(aLU.second, aLL.second);
          poi1Max = std::max(poi1Max, aRU.second);
          poi1Max = std::max(poi1Max, aRL.second);

          poi2Min = std::min(bLU.second, bLL.second);
          poi2Min = std::min(poi2Min, bRU.second);
          poi2Min = std::min(poi2Min, bRL.second);

          poi2Max = std::max(bLU.second, bLL.second);
          poi2Max = std::max(poi2Max, bRU.second);
          poi2Max = std::max(poi2Max, bRL.second);
        } else {
          // Use x axis
          poi1Min = std::min(aLU.first, aLL.first);
          poi1Min = std::min(poi1Min, aRU.first);
          poi1Min = std::min(poi1Min, aRL.first);

          poi1Max = std::max(aLU.first, aLL.first);
          poi1Max = std::max(poi1Max, aRU.first);
          poi1Max = std::max(poi1Max, aRL.first);

          poi2Min = std::min(bLU.first, bLL.first);
          poi2Min = std::min(poi2Min, bRU.first);
          poi2Min = std::min(poi2Min, bRL.first);

          poi2Max = std::max(bLU.first, bLL.first);
          poi2Max = std::max(poi2Max, bRU.first);
          poi2Max = std::max(poi2Max, bRL.first);
        }


        CGALPoint aPolyPoints[] = { posToCGAL(aLU), posToCGAL(aLL), posToCGAL(aRL), posToCGAL(aRU) };
        CGALPoint bPolyPoints[] = { posToCGAL(bLU), posToCGAL(bLL), posToCGAL(bRL), posToCGAL(bRU) };


        std::cout << "A: polygon through (" << aLU.first << "," << aLU.second << ") (" << aLL.first << "," << aLL.second << ") (" << aRL.first << "," << aRL.second << ") (" << aRU.first << "," << aRU.second << ")";
        std::cout << "B: polygon through (" << bLU.first << "," << bLU.second << ") (" << bLL.first << "," << bLL.second << ") (" << bRL.first << "," << bRL.second << ") (" << bRU.first << "," << bRU.second << ")";


        Polygon_2 aPoly = Polygon_2(aPolyPoints, aPolyPoints+4);
        Polygon_2 bPoly = Polygon_2(bPolyPoints, bPolyPoints+4);

        bool result = CGAL::do_intersect(aPoly, bPoly);
*/
/*
        return ((poi1Min <= poi2Max) && (poi1Max >= poi2Min));
        //return result;
    }
*/
    static bool rectanglesOverlap(Position luA, Position ruA, Position llA, Position rlA, Position luB, Position ruB, Position llB, Position rlB) {
        CGALPoint aPolyPoints[] = { posToCGAL(luA), posToCGAL(llA), posToCGAL(rlA), posToCGAL(ruA) };
        //CGALPoint bPolyPoints[] = { posToCGAL(luB), posToCGAL(llB), posToCGAL(rlB), posToCGAL(ruB) };
        Polygon_2 aPoly = Polygon_2(aPolyPoints, aPolyPoints+4);
        //Polygon_2 bPoly = Polygon_2(bPolyPoints, bPolyPoints+4);

        /*
        std::cout << "Testing Overlap:";
        std::cout << "A: polygon through (" << luA.first << "," << luA.second << ") (" << llA.first << "," << llA.second << ") (" << rlA.first << "," << rlA.second << ") (" << ruA.first << "," << ruA.second << ")";
        std::cout << "B: polygon through (" << luB.first << "," << luB.second << ") (" << llB.first << "," << llB.second << ") (" << rlB.first << "," << rlB.second << ") (" << ruB.first << "," << ruB.second << ")";
        */

        bool result = aPoly.bounded_side(posToCGAL(luB)) != CGAL::ON_UNBOUNDED_SIDE ||
                aPoly.bounded_side(posToCGAL(rlB)) != CGAL::ON_UNBOUNDED_SIDE ||
                aPoly.bounded_side(posToCGAL(ruB)) != CGAL::ON_UNBOUNDED_SIDE ||
                aPoly.bounded_side(posToCGAL(llB)) != CGAL::ON_UNBOUNDED_SIDE;

        //std::cout << "Result is: " << result;
        return result;

        //std::vector<Polygon_with_holes_2> result;
        //CGAL::intersection(aPoly, bPoly, std::inserter(result, result.begin()));

        //return result.size() > 0;

        /*
        bool result = CGAL::do_intersect(aPoly, bPoly);
        return result;
        */
    }

    static CGALPoint posToCGAL(Position pos) {
        return CGALPoint(pos.first, pos.second);
    }

    static IntPoint posToClipper(Position pos) {
      return IntPoint(pos.first * 100000, pos.second * 100000);
    }

    // TODO this only works for aligned rectangles!
    static bool aContainsB(Position luA, Position ruA, Position llA, Position rlA, Position luB, Position ruB, Position llB, Position rlB) {
        CGALPoint aPolyPoints[] = { posToCGAL(luA), posToCGAL(llA), posToCGAL(rlA), posToCGAL(ruA) };
        CGALPoint bPolyPoints[] = { posToCGAL(luB), posToCGAL(llB), posToCGAL(rlB), posToCGAL(ruB) };
        Polygon_2 aPoly = Polygon_2(aPolyPoints, aPolyPoints+4);
        Polygon_2 bPoly = Polygon_2(bPolyPoints, bPolyPoints+4);

        // TODO this should be enough...
        bool result = aPoly.bounded_side(posToCGAL(luB)) != CGAL::ON_UNBOUNDED_SIDE &&
                aPoly.bounded_side(posToCGAL(rlB)) != CGAL::ON_UNBOUNDED_SIDE;
        /*
        bool result = CGAL::bounded_side_2(aPolyPoints, aPolyPoints+4, posToCGAL(luB)) &&
                CGAL::bounded_side_2(aPolyPoints, aPolyPoints+4, posToCGAL(ruB)) &&
                CGAL::bounded_side_2(aPolyPoints, aPolyPoints+4, posToCGAL(rlB)) &&
                CGAL::bounded_side_2(aPolyPoints, aPolyPoints+4, posToCGAL(llB));
        */
        return result;

    }

private:
};

#endif // COORDINATEUTIL_H
