#include "nooverlapschecker.h"

#include "config.h"

#include "util/coordinateutil.h"

NoOverlapsChecker::NoOverlapsChecker(Camera *camera, Map *map, Trajectory *trajectory):
  camera(camera), map(map), trajectory(trajectory)
{}

bool
NoOverlapsChecker::checkPair(const RotatingPOI &rpoi1, const RotatingPOI &rpoi2) {
  //bool result = !CoordinateUtil::rpoiOverlap(&rpoi1, &rpoi2);


  double intersectionArea = CoordinateUtil::rpoiOverlapArea(&rpoi1, &rpoi2);
  bool result = !(intersectionArea >= (rpoi1.getHeight() * rpoi1.getWidth() * 0.01));

/*
  if (!result) {
    std::cout << "~~~~~ Should be in conflict: " << rpoi1.getPoi()->getLabel() << "(" << rpoi1.getPoi()->getId() << ")" << " vs " << rpoi2.getPoi()->getLabel() << "(" << rpoi2.getPoi()->getId() << ")" << "\n";
    std::cout << "~~~~~ ";
    rpoi1.dbgPrintToWolfram();
    std::cout << "   ---    ";
    rpoi2.dbgPrintToWolfram();
    std::cout << "\n";
    std::cout << "Area of overlap: " << intersectionArea << " / Area of POI 1: " << (rpoi1.getHeight() * rpoi1.getWidth()) << "\n";
  }
  */

  return result;
}


bool
NoOverlapsChecker::checkAt(int point, std::set<std::set<POI *>> *problems)
{
  std::set<std::set<POI *>> conflicts;
  std::set<POI *> visible;
  conflicts = this->camera->getConflictsAt(point);
  visible = this->camera->getVisibleAt(point);

  CarPosition carpos = this->trajectory->interpolatePosition(point);
  std::vector<RotatingPOI> all_rpois = this->camera->getPOIsForCar(carpos, true);
  std::vector<RotatingPOI> rpois;

  std::copy_if(all_rpois.begin(), all_rpois.end(), std::back_inserter(rpois), [&](RotatingPOI rpoi){
    return (visible.find(rpoi.getPoi()) != visible.end());
  });

  bool result = true;
  for (std::vector<RotatingPOI>::const_iterator a = rpois.begin(), b, end = rpois.end(); a != end; ++a)
  {
    for (b = (a+1); b != end; ++b) {

      POI *poi1 = a->getPoi();
      POI *poi2 = b->getPoi();

      if (conflicts.find(std::set<POI*>({poi1, poi2})) == conflicts.end()) {
        bool thisResult = this->checkPair(*a, *b);
        std::set<POI *> participants({poi1, poi2});

        if ((this->last_seen.find(participants) != this->last_seen.end()) &&
            (this->last_seen[participants] == (point - 1))) {
          if ((problems != nullptr) && (!thisResult)) {
            problems->insert(std::set<POI *>({poi1, poi2}));
          }
          result = false;


          std::cout << "~~~~~ Should be in conflict: " << a->getPoi()->getLabel() << "(" << a->getPoi()->getId() << ")" << " vs " << b->getPoi()->getLabel() << "(" << b->getPoi()->getId() << ")" << "\n";
          std::cout << "~~~~~ ";
          a->dbgPrintToWolfram();
          std::cout << "   ---    ";
          b->dbgPrintToWolfram();
          std::cout << "\n";
          //std::cout << "Area of overlap: " << intersectionArea << " / Area of POI 1: " << (a->getHeight() * a->getWidth()) << "\n";

        }
        this->last_seen[participants] = point;

      }
    };
  }

  return result;
}

bool
NoOverlapsChecker::check()
{
  bool result = true;
  std::set<std::set<POI *>> problems;
  for (int point = 0 ; point <= this->trajectory->getLength() ; point += CHECK_STEP) {
    std::cout << "Checking at " << point << " \\ " << trajectory->getLength() <<  "\n";
    std::set<std::set<POI *>> newProblems;
    result &= this->checkAt(point, &newProblems);

    if (newProblems != problems) {
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
      std::cout << "Problems changed at " << point << ":\n";

      std::vector<std::set<POI*>> addedProblems;
      std::set_difference(newProblems.begin(), newProblems.end(), problems.begin(), problems.end(), std::back_inserter(addedProblems));
      std::vector<std::set<POI*>> removedProblems;
      std::set_difference(problems.begin(), problems.end(), newProblems.begin(), newProblems.end(), std::back_inserter(removedProblems));

      std::cout << "==== Added Problems:\n";

      for (std::set<POI *> conflict : addedProblems) {
        POI *poi1 = *(conflict.begin());
        POI *poi2 = *(++(conflict.begin()));
        std::cout << poi1->getLabel() << " vs " << poi2->getLabel() << "\n";
      }

      std::cout << "==== Removed Problems:\n";

      for (std::set<POI *> conflict : removedProblems) {
        POI *poi1 = *(conflict.begin());
        POI *poi2 = *(++(conflict.begin()));
        std::cout << poi1->getLabel() << " vs " << poi2->getLabel() << "\n";
      }

      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
      problems = newProblems;
    }
  }

  return result;
}
