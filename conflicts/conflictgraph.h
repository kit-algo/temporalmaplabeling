#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include "util/setrtree.h"
#include "conflicts/camera.h"
#include "heuristics/heuristic.h"

/* Using bundled properties here. */
/*
 * Expanded Conflict Graph
 */

 struct ECGEdgeProperties {
   bool conflict_edge;
   std::vector<Interval> intervals;
 };

 struct ECGVertexProperties {
   double weight;
   Interval interval;
   POI *poi;
   Interval origVisibility;
   double compound_weight;
   bool minimal;
 };

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, ECGVertexProperties, ECGEdgeProperties > ExpandedConflictGraphBase;

class ExpandedConflictGraph : public ExpandedConflictGraphBase {
public:
  typedef boost::graph_traits<ExpandedConflictGraphBase>::vertex_descriptor vertex_t;
  typedef boost::graph_traits<ExpandedConflictGraphBase>::edge_descriptor edge_t;

  static ExpandedConflictGraph *fromConflicts(ConflictIntervals &conflicts, VisibilityIntervals visibilityIntervals, Heuristic::ModelType mtype, bool compound);
  static void writeToFile(const char *fileName, const ExpandedConflictGraph &g);

  void compute_compound_weight(vertex_t v, const std::set<vertex_t> &blocked);
  void compute_compound_weights(const std::set<vertex_t> &blocked);

private:
  bool compound;
};

#endif
