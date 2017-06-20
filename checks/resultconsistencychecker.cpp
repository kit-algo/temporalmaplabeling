#include "resultconsistencychecker.h"

#include "util/setrtree.h"


ResultConsistencyChecker::ResultConsistencyChecker(SelectionIntervals *selected, Camera *camera, Map *map, int k):
  selected(selected), camera(camera), map(map), k(k)
{}

void
ResultConsistencyChecker::prepare() {
  for (auto selection : this->selected->queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    POI * poi = selection.second;
    Interval interval = selection.first;
    this->poi_selected[poi].push_back(interval);
  }

  for (auto visibility : this->camera->getVisibilityIntervals().queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    POI * poi = visibility.second;
    Interval interval = visibility.first;
    this->poi_visible[poi].push_back(interval);
  }

  for (auto conflictEntry : this->camera->getConflictIntervals().queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    std::set<POI *> participants = conflictEntry.second;
    Interval conflict = conflictEntry.first;

    assert(participants.size() == 2);
    POI *poi1 = *(participants.begin());
    POI *poi2 = *(++(participants.begin()));
    assert (poi1->getId() != poi2->getId());

    this->poi_conflicts[std::set<POI *>({poi1, poi2})].push_back(conflict);
  }

  for (auto poi : this->map->getPOI()) {
    std::sort(this->poi_selected[poi].begin(), this->poi_selected[poi].end(), compareIntervals);
    std::sort(this->poi_visible[poi].begin(), this->poi_visible[poi].end(), compareIntervals);

    for (auto poi2 : this->map->getPOI()) {
      std::sort(this->poi_conflicts[std::set<POI*>({poi, poi2})].begin(), this->poi_conflicts[std::set<POI*>({poi, poi2})].end(), compareIntervals);
    }
  }

}

void
ResultConsistencyChecker::check() {
  this->prepare();
  this->check_inside();
  this->check_outside();
  if (this->k > 0) {
    this->check_restriction();
  }
}

void
ResultConsistencyChecker::check_restriction() {
  std::vector<std::pair<double, bool>> selection_events;

  for (auto entry : this->poi_selected) {
    for (auto interval : entry.second) {
      selection_events.push_back(std::make_pair(bg::get<0>(interval.first), true));
      selection_events.push_back(std::make_pair(bg::get<0>(interval.second), false));
    }
  }

  std::sort(selection_events.begin(), selection_events.end(), [&](const std::pair<double,bool> &e1, const std::pair<double,bool> &e2) {
    if (e1.first != e2.first) {
      return e1.first < e2.first;
    }

    return ((!e1.second) && (e2.second));
  });

  int counter = 0;
  for (auto entry : selection_events) {
    bool start;
    double dist;
    std::tie(dist, start) = entry;

    if (start) {
      counter++;
      assert(counter <= this->k);
    } else {
      counter--;
      assert(counter >= 0);
    }
  }
}

bool
ResultConsistencyChecker::triple_check(Interval i1, Interval i2, Interval i3)
{
  // Step 1: Construct cut of i1 and i2
  double i12_start = std::max(bg::get<0>(i1.first), bg::get<0>(i2.first));
  double i12_end = std::min(bg::get<0>(i1.second), bg::get<0>(i2.second));

  if (i12_start >= i12_end) {
    return false; // i1 and i2 do not overlap
  }

  return !((i12_start >= bg::get<0>(i3.second)) || (i12_end <= bg::get<0>(i3.first)));
}

void
ResultConsistencyChecker::check_outside() {
  for (std::set<POI *> display_pair : this->camera->getDisplayPairs().query(bgi::satisfies([](Interval const&){ return true; }))) {
    assert(display_pair.size() == 2);
    POI *poi1 = *(display_pair.begin());
    POI *poi2 = *(++(display_pair.begin()));
    assert (poi1->getId() != poi2->getId());

    auto sel1_it = this->poi_selected[poi1].begin();
    auto sel2_it = this->poi_selected[poi2].begin();
    auto conf_it = this->poi_conflicts[std::set<POI*>({poi1, poi2})].begin();


    while ((sel1_it != this->poi_selected[poi1].end()) &&
           (sel2_it != this->poi_selected[poi2].end()) &&
           (conf_it != this->poi_conflicts[std::set<POI*>({poi1, poi2})].end())) {

      // Fast forward to the next conflict that concerns only these two POIs

      if(this->triple_check(*sel1_it, *sel2_it, *conf_it)) {
        std::cout << "!!!!!!!!!!!!!! INCONSISTENCY DETECTED !!!!!!!!!!!!!!!!!!\n";
        std::cout << "Kind:          Triple-Overlap\n";
        std::cout << "POI 1:         " << poi1->getId() << " (" << poi1->getLabel() << ")\n";
        std::cout << "Visibility 1:  " << *sel1_it << "\n";
        std::cout << "POI 2:         " << poi2->getId() << " (" << poi2->getLabel() << ")\n";
        std::cout << "Visibility 2:  " << *sel2_it << "\n";
        std::cout << "Conflict:      " << *conf_it << "\n";
        std::cout << "!!!!!!!!!!!!!! INCONSISTENCY DETECTED !!!!!!!!!!!!!!!!!!\n";
        assert(false);
      };

      Interval min_val = std::min(std::min(*sel1_it, *sel2_it, compareIntervals), *conf_it, compareIntervals);
      if (*sel1_it == min_val) {
        sel1_it++;
      } else if (*sel2_it == min_val) {
        sel2_it++;
      } else {
        assert(*conf_it == min_val);
        conf_it++;
      }
    }
  }
}

void
ResultConsistencyChecker::check_inside() {
  for (auto poi : this->map->getPOI()) {
    auto vis_it = this->poi_visible[poi].begin();
    auto sel_it = this->poi_selected[poi].begin();

    while ((sel_it != this->poi_selected[poi].end()) && (vis_it != this->poi_visible[poi].end())) {
      Interval vis_interval = *(vis_it);
      Interval sel_interval = *(sel_it);

      bool done = false;
      while (bg::get<0>(vis_interval.second) < bg::get<0>(sel_interval.first)) {
        // Advance visibilities as long as they end before the current selection
        vis_it++;
        if (vis_it == this->poi_visible[poi].end()) {
          done = true;
          break;
        }
        assert(bg::get<0>((*vis_it).first) > bg::get<0>(vis_interval.second)); // Visibilities are disjoint
        vis_interval = *(vis_it);
      }
      if (done) {
        break;
      }
      
      // Selection now must with the visibility -> it must start and end within the visibility!
      assert(bg::get<0>(sel_interval.first) >= bg::get<0>(vis_interval.first));
      assert(bg::get<0>(sel_interval.second) <= bg::get<0>(vis_interval.second));

      // Advance to the next visibility - this one may not have more selections
      vis_it++;
      assert((vis_it == this->poi_visible[poi].end()) || (bg::get<0>((*vis_it).first) > bg::get<0>(vis_interval.second))); // Visibilities are disjoint
      //vis_interval = *(vis_it);

      sel_it++;
      assert((sel_it == this->poi_selected[poi].end()) || (bg::get<0>((*sel_it).first) > bg::get<0>(sel_interval.second))); // Selections are disjoint
      //sel_interval = *(sel_it);
    }
  }
}
