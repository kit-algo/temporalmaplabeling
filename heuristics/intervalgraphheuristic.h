#ifndef INTERVALGRAPHHEURISTIC_H
#define INTERVALGRAPHHEURISTIC_H

#include <limits>

#include "heuristic.h"
#include "conflicts/conflictgraph.h"

#include <boost/icl/interval_map.hpp>

typedef boost::icl::interval_map<double, std::set<POI *>> BlockMap;
typedef boost::icl::interval<double> MapInterval;

class IGHeuristic : public Heuristic {

public:
  enum Mode {AM1,AM2,AM3};
    IGHeuristic(ExpandedConflictGraph g, Mode mode = AM1, int k = std::numeric_limits<int>::max());
  virtual SelectionIntervals * getLabelIntervals();
  void run();

private:
  ExpandedConflictGraph make_copy();

  const ExpandedConflictGraph graph;

  SelectionIntervals *labelIntervals;
  ModelType mtype;
  Mode  mode;
  int k;
};

#endif // INTERVALGRAPHHEURISTIC_H
