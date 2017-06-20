#include "trajectoryfilter.h"

#include "map/projection.h"
#include "util/coordinateutil.h"
#include "camera.h"

#include "config.h"

#include <QString>

TrajectoryFilter::TrajectoryFilter(Trajectory *trajectory, Camera *camera, Map *map):
trajectory(trajectory),
camera(camera),
map(map)
{
}

ConflictIntervals TrajectoryFilter::getDisplayPairs()
{
  if (this->displayPairs.size() == 0) {
    this->computeVisiblePOI();
  }

  return this->displayPairs;
}

ConflictIntervals
TrajectoryFilter::getConflicts()
{
  return this->conflicts;
}

void TrajectoryFilter::computeVisiblePOI()
{
  this->visiblePOI.clear();
  this->visibilityIntervals.clear();

  std::vector<double> openSince;
  openSince.resize(POI::getMaxIntenalId(), -1);

  std::vector<InterpolationStep> interpolation_steps = this->camera->getInterpolationPoints();
  int total = interpolation_steps.size();
  int i = 0;
  std::cout << "Interpolating..\n";

  std::map<std::set<POI *>, double> open_conflicts;
  this->conflicts.clear();

  double last_dist = 0;
  for (InterpolationStep step : interpolation_steps) {
    double dist = step.dist;

    i++;
    std::cout << "\x1b[A" << "Interpolation: Step " << i << " / " << total << "\n";

    RotatingPOI viewport = step.viewport;

    // Directly rotate this...
    Position vpLU = CoordinateUtil::rotatePos(viewport.getCorner(true, true), -1 * viewport.getRotation());
    Position vpLL = CoordinateUtil::rotatePos(viewport.getCorner(false, true), -1 * viewport.getRotation());
    Position vpRU = CoordinateUtil::rotatePos(viewport.getCorner(true, false), -1 * viewport.getRotation());
    Position vpRL = CoordinateUtil::rotatePos(viewport.getCorner(false, false), -1 * viewport.getRotation());

    #ifdef CONSISTENCY_CHECKS
    Position unrotated_vpLU = viewport.getCorner(true, true);
    Position unrotated_vpLL = viewport.getCorner(false, true);
    Position unrotated_vpRU = viewport.getCorner(true, false);
    Position unrotated_vpRL = viewport.getCorner(false, false);
    #endif

    std::vector<std::pair<RotatingPOI, QRectF>> visible_poi;
    std::set<POI *> lost_pois;
    for (auto rpoi : this->camera->getPOIsForCar(step.carpos, true)) {
      bool dbg_enable = false;

      if (rpoi.getPoi()->getId() == DBG_POI_1) {
        dbg_enable = true;
      }

      assert (rpoi.getRotation() == viewport.getRotation());
      // rotate back!
      Position pLU = CoordinateUtil::rotatePos(rpoi.getCorner(true, true), -1 * rpoi.getRotation());
      Position pLL = CoordinateUtil::rotatePos(rpoi.getCorner(false, true), -1 * rpoi.getRotation());
      Position pRU = CoordinateUtil::rotatePos(rpoi.getCorner(true, false), -1 * rpoi.getRotation());
      Position pRL = CoordinateUtil::rotatePos(rpoi.getCorner(false, false), -1 * rpoi.getRotation());

      #ifdef CONSISTENCY_CHECKS
      Position unrotated_pLU = rpoi.getCorner(true, true);
      Position unrotated_pLL = rpoi.getCorner(false, true);
      Position unrotated_pRU = rpoi.getCorner(true, false);
      Position unrotated_pRL = rpoi.getCorner(false, false);
      #endif

      bool overlap_vertical = ((pLU.first <= vpRU.first) && (pRU.first >= vpLU.first));
      bool overlap_horizontal = ((pLL.second <= vpLU.second) && (pLU.second >= vpLL.second));

      #ifdef CONSISTENCY_CHECKS
      assert((overlap_vertical && overlap_horizontal) == CoordinateUtil::rectanglesOverlap(unrotated_vpLU, unrotated_vpRU, unrotated_vpLL, unrotated_vpRL, unrotated_pLU, unrotated_pRU, unrotated_pLL, unrotated_pRL));
      #endif

      if (overlap_vertical && overlap_horizontal) {
        bool contain_horizontal = ((pLU.first >= vpLU.first) && (pRU.first <= vpRU.first));
        bool contain_vertical = ((pLU.second <= vpLU.second) && (pLL.second >= vpLL.second));

        if (contain_vertical && contain_horizontal) {
          this->visiblePOI.insert(rpoi.getPoi());
        }

        if (openSince[rpoi.getPoi()->getInternalId()] == -1) {
          openSince[rpoi.getPoi()->getInternalId()] = dist;
        }

        visible_poi.push_back(std::make_pair(rpoi, QRectF(QPointF(pLU.first, pLU.second), QPointF(pRL.first, pRL.second))));

      } else {
        if (openSince[rpoi.getPoi()->getInternalId()] != -1) {
          double start = openSince[rpoi.getPoi()->getInternalId()];

          double end = last_dist;

          if (end > start) {
            this->visibilityIntervals.insert(make_interval(start, end), rpoi.getPoi());
          }
          //this->visibilityIntervals.dbg_output();
          //this->visibilityIntervals.add(std::make_pair(Interval(start, end), std::set<POI *>({rpoi.getPoi()})));
          //this->visibilityIntervals.push_back(std::make_tuple(start, end, rpoi.getPoi()));
          openSince[rpoi.getPoi()->getInternalId()] = -1;

          lost_pois.insert(rpoi.getPoi());
        }
      }
    }

    std::vector<std::set<POI *>> to_close;


    for (auto ongoing_conflict : open_conflicts) {
      const std::set<POI *> &participants = ongoing_conflict.first;

      if ((lost_pois.find(*(participants.begin())) != lost_pois.end()) ||
          (lost_pois.find(*(++(participants.begin()))) != lost_pois.end()))
      {
        to_close.push_back(participants);
      }
    }

    for (auto participants : to_close) {
      double start = open_conflicts[participants];
      double end = last_dist;
      this->conflicts.insert(make_interval(start, end), participants);
      open_conflicts.erase(participants);
    }

    //std::cout << "===============================\n";
    //std::cout << "Testing at " << dist << "\n";

    for (int i  = 0 ; i < visible_poi.size(); i++) {
      for (int j = i+1 ; j < visible_poi.size(); j++) {
        std::pair<RotatingPOI, QRectF> &pair1 = visible_poi[i];
        std::pair<RotatingPOI, QRectF> &pair2 = visible_poi[j];

        bool dbg_enable = false;
        /*
        if (((pair1.first.getPoi()->getId() == DBG_POI_2) && (pair2.first.getPoi()->getId() == DBG_POI_1)) ||
            ((pair2.first.getPoi()->getId() == DBG_POI_2) && (pair1.first.getPoi()->getId() == DBG_POI_1))) {
          dbg_enable = true;
          std::cout << "Testing " << pair1.first.getPoi()->getLabel() << " vs " << pair2.first.getPoi()->getLabel() << "\n";
        }
        */

        if (pair1.second.intersects(pair2.second)) {
          if (dbg_enable){
            std::cout << "Do intersect!\n";
          }
          std::set<POI *> participants({pair1.first.getPoi(), pair2.first.getPoi()});
          if (open_conflicts.find(participants) == open_conflicts.end()) {
            open_conflicts[participants] = step.dist;
          }
        } else {
          if (dbg_enable) {
            std::cout << "Dont intersect!\n";
          }
          std::set<POI *> participants({pair1.first.getPoi(), pair2.first.getPoi()});
          if (open_conflicts.find(participants) != open_conflicts.end()) {
            double start = open_conflicts[participants];
            double end = last_dist;

            if (end > start) {
              this->conflicts.insert(make_interval(start, end), participants);
            }
            open_conflicts.erase(participants);
          }
        }

      }
    }

    last_dist = dist;
  }

  for (auto not_closed : open_conflicts) {
    std::set<POI *> participants = not_closed.first;
    double start = not_closed.second;
    double end = trajectory->getLength();

    if (end > start) {
      this->conflicts.insert(make_interval(start, end), participants);
    }
  }

  for (auto poi : this->map->getPOI()) {
    if (openSince[poi->getInternalId()] != -1) {
      double start = openSince[poi->getInternalId()];
      if ( this->trajectory->getLength() > start) {
        this->visibilityIntervals.insert(make_interval(start, this->trajectory->getLength()), poi);
      }
    }
  }

  std::cout << "Interpolation done.\n";

  std::set<POI *> openPOIs = {};
  std::priority_queue<std::pair<double, POI *>, std::vector<std::pair<double, POI *>>, std::greater<std::pair<double, POI *>>> closePoints;

  for (int i = 0 ; i < openSince.size() ; i++) {
    openSince[i] = -1;
  }

  for (auto visiblePair : this->visibilityIntervals.queryFullSorted(bgi::satisfies([](Interval const&){ return true; }))) {
    POI * nowOpen = visiblePair.second;
    Interval interval = visiblePair.first;

    double start = interval.first.get<0>();
    double end = interval.second.get<0>();

    if (closePoints.size() > 0) {
      std::pair<double, POI *> popPair = closePoints.top();
      std::map<POI *, double> closedAt;
      while (popPair.first < start) {
        openPOIs.erase(popPair.second);
        closedAt[popPair.second] = popPair.first;
        closePoints.pop();
        if (closePoints.size() == 0) {
          break;
        }
        popPair = closePoints.top();
      }

      // Make intervals for POIs that have both been closed
      for (auto closedIt : closedAt) {
        POI *poi1 = closedIt.first;
        double end1 = closedIt.second;
        double start1 = openSince[poi1->getInternalId()];

        for (auto closedIt2 : closedAt) {
          POI *poi2 = closedIt2.first;
          double end2 = closedIt2.second;
          double start2 = openSince[poi2->getInternalId()];

          if (poi1 == poi2) continue;

          double common_start = std::max(start1, start2);
          double common_end = std::min(end1, end2);

          this->displayPairs.insert(make_interval(common_start, common_end), {poi1, poi2});
        }
      }

      // Make intervals where only one of both has been closed
      for (auto closedIt : closedAt) {
        POI *poi1 = closedIt.first;
        double end1 = closedIt.second;
        double start1 = openSince[poi1->getInternalId()];

        for (auto openIt : openPOIs) {
          POI *poi2 = openIt;
          double start2 = openSince[poi2->getInternalId()];

          double common_start = std::max(start1, start2);
          double common_end = end1;

          this->displayPairs.insert(make_interval(common_start, common_end), {poi1, poi2});
        }
      }

      // *AFTER* using it above, clean out openSince
      for (auto closedIt : closedAt) {
        POI *poi1 = closedIt.first;
        openSince[poi1->getInternalId()] = -1;
      }
    }

    if (openPOIs.find(nowOpen) == openPOIs.end()) {
      openPOIs.insert(nowOpen);
      openSince[nowOpen->getInternalId()] = start;
      closePoints.push(std::make_pair(end, nowOpen));
    } else {
      throw "Double open!";
    }

    /*
    std::cout << "Visible from " << interval.lower() << " to " << interval.upper() << ": " << nowOpen.size();

    std::vector<POI *> wasClosed;
    std::set_difference(openPOIs.begin(), openPOIs.end(), nowOpen.begin(), nowOpen.end(), std::inserter(wasClosed, wasClosed.end()));
    for (auto closedPoi : wasClosed) {
    for (auto otherPoi : openPOIs) {
    if (closedPoi == otherPoi)
    continue;
    double start = std::max(openSince[otherPoi], openSince[closedPoi]);
    double end = interval.lower();
    this->displayPairs.add(std::make_pair(Interval(start, end), std::set<std::set<POI *>>({{closedPoi, otherPoi}})));
  }
}
// Clean the openSince *after* potentially using them above
for (auto closedPoi : wasClosed) {
openSince.erase(closedPoi);
}

std::vector<POI *> newOpen;
std::set_difference(nowOpen.begin(), nowOpen.end(), openPOIs.begin(), openPOIs.end(), std::inserter(newOpen, newOpen.end()));
for (auto newOpenPoi : newOpen) {
openSince.insert(std::make_pair(newOpenPoi, interval.lower()));
}

openPOIs = nowOpen;
*/
}

// Finally, close all that remain open

//assert(this->visibilityIntervals.size() > 0);
//double end = (--(this->visibilityIntervals.end()))->first.upper();

while (closePoints.size() > 0) {
  std::pair<double, POI *> popPair = closePoints.top();
  closePoints.pop();
  POI *poi1 = popPair.second;
  double end1 = popPair.first;
  double start1 = openSince[poi1->getInternalId()];

  openPOIs.erase(poi1);
  for (auto openIt : openPOIs) {
    POI *poi2 = openIt;
    double start2 = openSince[poi2->getInternalId()];

    double common_start = std::max(start1, start2);
    double common_end = end1;

    this->displayPairs.insert(make_interval(common_start, common_end), {poi1, poi2});
  }
  openSince[poi1->getInternalId()] = -1;
}

/*
for (auto openPoi : openPOIs) {
for (auto otherPoi : openPOIs) {
if (otherPoi == openPoi)
continue;
double start = std::max(openSince[otherPoi], openSince[openPoi]);
this->displayPairs.add(std::make_pair(Interval(start, end), std::set<std::set<POI *>>({{openPoi, otherPoi}})));
}
}
*/
}
VisibilityIntervals TrajectoryFilter::getVisibilityIntervals() const
{
  return visibilityIntervals;
}
