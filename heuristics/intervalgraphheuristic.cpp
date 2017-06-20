#include "intervalgraphheuristic.h"
#include "utils.h"
#include "config.h"

#include "conflicts/conflictgraph.h"

#include <queue>
#include <cassert>

#include <boost/graph/copy.hpp>


IGHeuristic::IGHeuristic(ExpandedConflictGraph g, Mode mode, int k):
  graph(g), labelIntervals(nullptr), mode(mode), k(k)
{
  if (this->k < 1) {
    this->k = std::numeric_limits<int>::max();
  }
}

SelectionIntervals *
IGHeuristic::getLabelIntervals()
{
  return this->labelIntervals;
}

ExpandedConflictGraph
IGHeuristic::make_copy()
{
  ExpandedConflictGraph result = this->graph;
  return result;
}

void
IGHeuristic::run()
{
  ExpandedConflictGraph g = this->make_copy();

  if (this->labelIntervals != nullptr) {
    delete this->labelIntervals;
  }
  this->labelIntervals = new SelectionIntervals();

  // set of presence intervals that can still be chosen
  std::set<ExpandedConflictGraph::vertex_t> remaining;

  // at the beginning all vertices can be chosen.
  auto vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
      if(g[*vit].weight > 0){
            remaining.insert(*vit);
      }
  }

  int iteration =0;
  while(!remaining.empty() && iteration < k){
      iteration++;
      std::vector<ExpandedConflictGraph::vertex_t> vertices;
      std::vector<WeightedInterval> intervals;
        int id =0;

    // create weighted intervals
    for(const ExpandedConflictGraph::vertex_t &v : remaining){
        Interval & visInterval = g[v].interval;
        intervals.push_back(WeightedInterval(id,bg::get<0>(visInterval.first),
                                             bg::get<0>(visInterval.second),g[v].weight));
        vertices.push_back(v);
        id++;

    }



    // compute the maximum weight independent set of the intervals
    std::vector<int> mwis = computeMWIS(intervals);

    assert(areIndependent(intervals,mwis));

    // For each interval that is contained mvis
    // - "remove" it from the graph
    // - according the given model, remove the neighbors or adapt their presence intervals.
    for(int index : mwis){
        const WeightedInterval & wi = intervals[index];

        ExpandedConflictGraph::vertex_t &v = vertices[wi.id()];

        this->labelIntervals->insert(make_interval(wi.start(),wi.end()), g[v].poi);


        remaining.erase(v);


        ExpandedConflictGraph::out_edge_iterator current, last;
        std::tie(current, last) = boost::out_edges(v, g);

        std::for_each(current, last,
          [&](ExpandedConflictGraph::edge_descriptor it)
            {
              ExpandedConflictGraph::vertex_t n = boost::target(it,g);
              Interval & visInterval = g[n].interval;

              assert(!intervals.empty());

              if(this->mode == AM1){ // in case of AM1 we just remove all conflicting presence intervals.
                  remaining.erase(n);
              }else if(this->mode == AM2){ // shorten all conflicting presence intervals to their longest prefix that is conflict-free with v
                  const std::vector<Interval> & intervals = g[it].intervals;

                  const Interval & front = intervals.front();

                  if(bg::get<0>(visInterval.first) + MINIMUM_SELECTION_LENGTH < std::min(bg::get<0>(front.first),bg::get<0>(visInterval.second))){ // there is a prefix.
                     g[n].interval = make_interval(bg::get<0>(visInterval.first),
                                                  std::min(bg::get<0>(front.first),bg::get<0>(visInterval.second)));
                     g[n].weight = std::min(bg::get<0>(front.first),bg::get<0>(visInterval.second))-bg::get<0>(visInterval.first);
                  }else{ // if there is no prefix then just delete the interval.
                      remaining.erase(n);
                  }

              }else if(this->mode == AM3){
                 const std::vector<Interval> & intervals = g[it].intervals;
                 Interval bestChoice = make_interval(0,0); // best pre-, in- or suffix that is conlfict free
                 assert(bg::get<0>(bestChoice.second)-bg::get<0>(bestChoice.first) >= 0);

                 // consider the first interval: it may allow a prefix of n
                 const Interval & front = intervals.front();
                 if(bg::get<0>(visInterval.first) < std::min(bg::get<0>(front.first),bg::get<0>(visInterval.second))){
                     bestChoice = make_interval(bg::get<0>(visInterval.first),
                                                std::min(bg::get<0>(front.first),bg::get<0>(visInterval.second)));
                 }


                 for(size_t i=1; i < intervals.size(); ++i){
                     const Interval & i1 = intervals[i-1];
                     const Interval & i2 = intervals[i];

                     double begin = std::max(bg::get<0>(visInterval.first),bg::get<0>(i1.second));
                     double end   = std::min(bg::get<0>(visInterval.second),bg::get<0>(i2.first));
                     if(bg::get<0>(bestChoice.second)-bg::get<0>(bestChoice.first) < end - begin){
                        bestChoice = make_interval(begin,end);
                     }
                 }

                 const Interval & back = intervals.back();

                 if(bg::get<0>(visInterval.second) > std::max(bg::get<0>(back.second),bg::get<0>(visInterval.first))){
                     if(bg::get<0>(bestChoice.second)-bg::get<0>(bestChoice.first) <
                             bg::get<0>(visInterval.second) - std::max(bg::get<0>(back.second),bg::get<0>(visInterval.first))){
                        bestChoice = make_interval(std::max(bg::get<0>(back.second),bg::get<0>(visInterval.first)),bg::get<0>(visInterval.second));
                     }
                 }

                 if(bg::get<0>(bestChoice.second)-bg::get<0>(bestChoice.first) >= MINIMUM_SELECTION_LENGTH){
                     g[n].interval = make_interval(bg::get<0>(bestChoice.first),
                                                   bg::get<0>(bestChoice.second));
                     g[n].weight = bg::get<0>(bestChoice.second)-bg::get<0>(bestChoice.first);
                 }else{
                     remaining.erase(n);
                 }
              }

             // std::cout << boost::target(it, g) << '\n';




            });
    }
}







//  /* Select visibilities! */
//  while (!vertices.empty()) {
//    std::pair<double, ConflictGraph::vertex_t> top = vertices.top();
//    vertices.pop();
//    ConflictGraph::vertex_t v = top.second;
//    if (blocked.find(v) != blocked.end()) {
//      continue;
//    }

//    Interval visInterval = g[v].interval;
//    POI *poi = g[v].poi;
//    this->labelIntervals->insert(visInterval, poi);

//    // Block neighbors!
//    auto neighbors = boost::adjacent_vertices(v, g);
//    for (auto nit = neighbors.first; nit != neighbors.second; ++nit) {
//      blocked.insert(*nit);
//    }
//  }
}
