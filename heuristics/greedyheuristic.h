#ifndef GREEDYHEURISTIC_H
#define GREEDYHEURISTIC_H

#include "heuristic.h"
#include "conflicts/conflictgraph.h"

#include <boost/icl/interval_map.hpp>
#include <boost/icl/split_interval_map.hpp>

/* Most interval map related stuff is taken straight from
 * http://www.boost.org/doc/libs/1_46_1/libs/icl/doc/html/boost_icl/examples/overlap_counter.html
 */
typedef boost::icl::interval_map<double, int> OverlapCounterT;

class GreedyHeuristic : public Heuristic {
public:
  GreedyHeuristic(ExpandedConflictGraph g, int k = -1);
  virtual SelectionIntervals * getLabelIntervals();
  void run();

private:
  ExpandedConflictGraph make_copy();

  const ExpandedConflictGraph graph;

  SelectionIntervals *labelIntervals;
  ModelType mtype;

  // stuff for k-Restriction
  int k;
  OverlapCounterT overlap_counter;
};

#endif
