#include "greedyheuristic.h"

#include <queue>
#include <cassert>

#include <boost/graph/copy.hpp>

GreedyHeuristic::GreedyHeuristic(ExpandedConflictGraph g, int k):
  graph(g), labelIntervals(nullptr), k(k)
{
}

SelectionIntervals *
GreedyHeuristic::getLabelIntervals()
{
  return this->labelIntervals;
}

ExpandedConflictGraph
GreedyHeuristic::make_copy()
{
  ExpandedConflictGraph result = this->graph;
  return result;
}

void dbg_print_overlaps(const OverlapCounterT& counter)
{
    for(OverlapCounterT::const_iterator it = counter.begin(); it != counter.end(); it++)
    {
        auto itv  = (*it).first;
        int overlaps_count = (*it).second;
        if(overlaps_count == 1)
            std::cout << "in interval " << itv << " intervals do not overlap" << std::endl;
        else
            std::cout << "in interval " << itv << ": "<< overlaps_count << " intervals overlap" << std::endl;
    }
}

void
GreedyHeuristic::run()
{
  ExpandedConflictGraph g = this->make_copy();

  if (this->labelIntervals != nullptr) {
    delete this->labelIntervals;
  }
  this->labelIntervals = new SelectionIntervals();

  /* Prepare the priority queue */
  //std::priority_queue<std::pair<double, ExpandedConflictGraph::vertex_t>> vertices;
  std::vector<ExpandedConflictGraph::vertex_t> queue;
  std::set<ExpandedConflictGraph::vertex_t> blocked;

  g.compute_compound_weights(blocked);

  auto vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    ExpandedConflictGraph::vertex_t v = *vit;
    queue.push_back(v);
  }

  std::make_heap(queue.begin(), queue.end(), [&](vertex_t &v1, vertex_t &v2) {
    return (g[v1].compound_weight < g[v2].compound_weight);
  });

  /* Select visibilities! */
  while (!queue.empty()) {
    ExpandedConflictGraph::vertex_t v = queue.front();
    double weight = g[v].compound_weight;
    std::pop_heap(queue.begin(), queue.end());
    queue.pop_back();

    if (blocked.find(v) != blocked.end()) {
      continue;
    }

    Interval visInterval = g[v].interval;

    bool k_reached = false;
    if (this->k > 0) {
      OverlapCounterT::interval_type map_interval = OverlapCounterT::interval_type::closed(bg::get<0>(visInterval.first), bg::get<0>(visInterval.second));

      /*
      std::cout << "||||||||||||||||||||||||||||||||||||||\n";
      std::cout << "Our interval is: " << bg::get<0>(visInterval.first) << " -> " << bg::get<0>(visInterval.second) << "\n";
      std::cout << "==== OVERLAP MAP: \n";
      dbg_print_overlaps(this->overlap_counter);
      std::cout << "----------- INTERSECTED OVERLAP MAP: \n";
      dbg_print_overlaps(this->overlap_counter & map_interval);
      */
      for (auto e : (this->overlap_counter & map_interval)) {
        if (e.second >= this->k) {
          k_reached = true;
          break;
        }
      }

      if (!k_reached) {
        this->overlap_counter += std::make_pair(map_interval, 1);
      }
    }
    if (k_reached) {
      continue;
    }

    POI *poi = g[v].poi;
    this->labelIntervals->insert(visInterval, poi);

    //std::cout << "Accepting vertex " << v << ": POI " << poi->getLabel() << " from " << bg::get<0>(visInterval.first) << " -> " <<  bg::get<0>(visInterval.second) << "\n";

    // Block neighbors!
    auto neighbors = boost::adjacent_vertices(v, g);
    std::set<ExpandedConflictGraph::vertex_t> recompute_at;
    for (auto nit = neighbors.first; nit != neighbors.second; ++nit) {
      //std::cout << "  ---> Blocking " << *nit << "\n";
      blocked.insert(*nit);
      auto blocked_neighbors = boost::adjacent_vertices(*nit, g);
      for (auto bnit = blocked_neighbors.first; bnit != blocked_neighbors.second; ++bnit) {
        recompute_at.insert(*bnit);
      }
    }

    for (auto recompute : recompute_at) {
      g.compute_compound_weight(recompute, blocked);
    }

    std::make_heap(queue.begin(), queue.end(), [&](vertex_t &v1, vertex_t &v2) {
      return (g[v1].compound_weight < g[v2].compound_weight);
    });
  }
}
