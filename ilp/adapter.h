#ifndef ADAPTER_H
#define ADAPTER_H

#include "ILP.h"

#include <map>
#include <set>

#include "../util/setrtree.h"
#include "../conflicts/camera.h"
#include "../heuristics/heuristic.h"


class ILPAdapter {
public:
  enum Result { OK, MEMORY, TIME };

  ILPAdapter(Map *map, VisibilityIntervals &visibilityIntervals, ConflictIntervals &conflicts, Heuristic::ModelType mtype, int k = -1, const char *dbg_filename = nullptr);
  void run();
  SelectionIntervals *getLabelIntervals();
  POI *poiForId(int id);
  Result getResult();
  double getBound();
  double getGap();

private:
  void determineMax();
  void makeVisibilityIntervals();
  void makeConflicts();

  double normalization_max;

  std::map<std::set<POI *>, int> conflictIDs;
  std::map<POI *, int> poiToID;
  std::map<int, POI *> idToPOI;

  ILP::Instance instance;

  Map *map;
  VisibilityIntervals &vi;
  ConflictIntervals &ci;

  SelectionIntervals *labelIntervals;

  Heuristic::ModelType mtype;

  Result res;
  double resBound;
  double gap;

  const char *dbg_filename;

  int k;
};

#endif
