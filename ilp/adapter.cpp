#include "adapter.h"
#include "config.h"

#include <gurobi_c++.h>

ILPAdapter::ILPAdapter(Map *map, VisibilityIntervals &visibilityIntervals, ConflictIntervals &conflicts, Heuristic::ModelType mtype, int k, const char* dbg_filename)
  : map(map), vi(visibilityIntervals), ci(conflicts), labelIntervals(nullptr), mtype(mtype), k(k), dbg_filename(dbg_filename)
{}

POI *
ILPAdapter::poiForId(int id) {
  return this->idToPOI[id];
}

void ILPAdapter::makeVisibilityIntervals()
{
  std::map<POI *, ILP::Intervals *> adaptedVI;
  int next_id = 0;

  for (auto intervalPOIpair : this->vi.queryFullSorted(bgi::satisfies([](Interval const&){ return true; }))) {
    Interval interval = intervalPOIpair.first;
    POI *poi = intervalPOIpair.second;

    int poi_id;
    if (this->poiToID.find(poi) != this->poiToID.end()) {
      poi_id = this->poiToID[poi];
    } else {
      poi_id = next_id++;
      this->poiToID[poi] = poi_id;
      this->idToPOI[poi_id] = poi;
    }

    if (adaptedVI.find(poi) == adaptedVI.end()) {
      adaptedVI.insert(make_pair(poi, new ILP::Intervals(poi_id)));
    }

#ifdef ENABLE_DEBUG
    if (poi->getId() == DBG_POI_1) {
      std::cout << "Adapting an interval for " << poi->getLabel() << " (ILP-ID " << this->poiToID[poi] << "): " << bg::get<0>(interval.first) << " -> " << bg::get<0>(interval.second) << "\n";
    }
#endif

    adaptedVI[poi]->add(bg::get<0>(interval.first) / this->normalization_max, bg::get<0>(interval.second) / this->normalization_max);
  }

  for (int i = 0 ; i < next_id ; i++) {
    POI * poi = this->idToPOI[i];

    this->instance.intervalsOfLabels->push_back(adaptedVI[poi]);
  }

#ifdef ENABLE_DEBUG
  std::cout << "ILP Normalization factor is: " << this->normalization_max << "\n";
  std::cout << "============= ILP INTERVALS ==================\n";
  for (int i = 0 ; i < next_id ; i++) {
    std::cout << "[" << i << "]: ";
    ILP::Intervals* thisPoiIntervals = this->instance.intervalsOfLabels->at(i);
    for (int j = 0 ; j < thisPoiIntervals->size() ; j += 2) {
        std::cout << "(" << (*thisPoiIntervals)[j] << " -> " << (*thisPoiIntervals)[j + 1] << "),";
    }
    std::cout << "\n";
  }
  std::cout << "============= END ILP INTERVALS ==================\n";
#endif
}

bool dbg_set_contains(std::set<POI *> s, int dbg_id);

void ILPAdapter::makeConflicts()
{
  this->conflictIDs.clear();
  std::map<int, std::pair<POI *, POI *>> reverseIDMap;
  int next_id = 0;

  std::map<int, ILP::Intervals *> adaptedCI;

  for (auto intervalConflictpair : this->ci.queryFullSorted(bgi::satisfies([](Interval const&){ return true; }))) {
    Interval interval = intervalConflictpair.first;
    std::set<POI *> conflict = intervalConflictpair.second;


    int id;
    if (this->conflictIDs.find(conflict) == this->conflictIDs.end()) {
      id = next_id++;
      this->conflictIDs.insert(make_pair(conflict, id));
      adaptedCI.insert(make_pair(id, new ILP::Intervals(id)));
      reverseIDMap[id] = make_pair(*(conflict.begin()), *(++(conflict.begin())));
    } else {
      id = this->conflictIDs[conflict];
    }

    adaptedCI[id]->add(bg::get<0>(interval.first) / this->normalization_max, bg::get<0>(interval.second) / this->normalization_max);
  }


  for (auto entry : adaptedCI) {
    int id = entry.first;

    this->instance.conflicts->push_back(new ILP::Conflict(this->poiToID[reverseIDMap[id].first], this->poiToID[reverseIDMap[id].second]));
    this->instance.intervalsOfConflicts->push_back(entry.second);
  }
}

VisibilityIntervals *
ILPAdapter::getLabelIntervals()
{
  return this->labelIntervals;
}

void
ILPAdapter::determineMax()
{
  this->normalization_max = 0.0;

  for (auto intervalPOIpair : this->vi.queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    Interval interval = intervalPOIpair.first;
    this->normalization_max = std::max(this->normalization_max, bg::get<0>(interval.second));
  }
}

double ILPAdapter::getBound() {
  return this->resBound * this->normalization_max;
}

double ILPAdapter::getGap() {
  return this->gap;
}

void ILPAdapter::run()
{
  this->determineMax();
  this->makeVisibilityIntervals();
  this->makeConflicts();
  this->instance.originalMax = this->normalization_max;

  std::vector<ILP::Intervals> result;
  Clock clock;
  bool timeLimitExeeded;

  if (this->labelIntervals != nullptr) {
    delete this->labelIntervals;
  }

  try {
    ILP::ILP ilp = ILP::ILP();
    switch (this->mtype) {
      case Heuristic::AM1:
        timeLimitExeeded = ! ilp.execute(&(this->instance), ILP::AM1, result, this->resBound, this->gap, clock, this->k, this, this->dbg_filename);
        break;
      case Heuristic::AM2:
        timeLimitExeeded = ! ilp.execute(&(this->instance), ILP::AM2, result, this->resBound, this->gap, clock, this->k, this, this->dbg_filename);
        break;
      case Heuristic::AM3:
        timeLimitExeeded = ! ilp.execute(&(this->instance), ILP::AM3, result, this->resBound, this->gap, clock, this->k, this, this->dbg_filename);
        break;
    }
  } catch (GRBException e) {
    std::cout << "!!!! EXCEPTION IN ILP !!!!!\n";
    std::cout << e.getMessage() << "\n";
    std::cout << "Code: " << e.getErrorCode() << "\n";
    switch (e.getErrorCode()) {
      case 10001:
        this->res = MEMORY;
        break;
    }
    std::cout << "\n";
    return;
  } catch (char const *e) {
    std::cout << "!!!! EXECPTION IN ILP !!!!!\n";
    std::cout << e;
    std::cout << "\n";
    return;
  }

  if (timeLimitExeeded) {
    cerr << "Time Limit exeeded!\n";
    this->res = TIME;
  } else {
    cerr << "Finished within time limit\n";
    this->res = OK;

    this->labelIntervals = new SelectionIntervals();

    for (ILP::Intervals inIntervals : result) {
      POI *poi = this->idToPOI[inIntervals.getId()];
      auto posIt = inIntervals.begin();
      while (posIt != inIntervals.end()) {
        double start = *posIt++;
        double end = *posIt++;

        #ifdef ENABLE_DEBUG
        if (poi->getId() == DBG_POI_1) {
          std::cout << "Selected Visibility for POI " << poi->getLabel() << ": " << start << " -> " << end << "\n";
        }
        #endif
        
        start *= this->normalization_max;
        end *= this->normalization_max;

        this->labelIntervals->insert(make_interval(start, end), poi);
      }
    }
  }
}
