#include "conflictgraph.h"

#include <fstream>
#include <ios>
#include <boost/graph/graphml.hpp>
#include <iostream>

#include "config.h"

#include "util/debugging.h"

void
ExpandedConflictGraph::compute_compound_weight(vertex_t v, const std::set<vertex_t> &blocked)
{
  if (!this->compound) {
    return;
  }

  double compound_weight = (*this)[v].weight;

  auto neighbors = boost::adjacent_vertices(v, *this);
  for (auto nit = neighbors.first; nit != neighbors.second; ++nit) {
    if (blocked.find(*nit) != blocked.end()) {
      compound_weight -= (*this)[*nit].weight;
    }
  }

  (*this)[v].compound_weight = compound_weight;
}

void
ExpandedConflictGraph::compute_compound_weights(const std::set<vertex_t> &blocked)
{
  if (!this->compound) {
    return;
  }

  auto vs = boost::vertices(*this);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    if (blocked.find(*vit) != blocked.end()) {
      (*this)[*vit].compound_weight = std::numeric_limits<double>::lowest();
    } else {
      this->compute_compound_weight(*vit, blocked);
    }
  }
}

ExpandedConflictGraph *
ExpandedConflictGraph::fromConflicts(ConflictIntervals &conflicts, VisibilityIntervals visibilityIntervals, Heuristic::ModelType mtype, bool compound)
{
  std::map<std::pair<double, double>, ExpandedConflictGraph::vertex_t> representants[POI::getMaxIntenalId()];

  //std::map<POI *, std::map<std::pair<double, double>, ExpandedConflictGraph::vertex_t>> representants;
  ExpandedConflictGraph *g = new ExpandedConflictGraph();
  g->compound = compound;
  long edges_total = 0;
  std::set<std::set<vertex_t>> presence_sets;

  //std::map<std::set<vertex_t>, std::pair<bool, std::vector<Interval>>> edges_to_add;
  std::vector<std::pair<std::set<vertex_t>, Interval>> conflict_edges;

  for (auto visibility : visibilityIntervals.queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    Interval visInterval = visibility.first;
    POI *poi = visibility.second;

    std::set<std::pair<double, POI *>> conflictStarts;
    std::set<std::pair<double, POI *>> conflictEnds;

    /* Step 1: Prepare lists of conflict starts and ends within this visibility */
    for (auto conflict : conflicts.queryFull(bgi::intersects(visInterval))) {
      Interval conflictInterval = conflict.first;
      std::set<POI *> participants = conflict.second;
      if (participants.find(poi) == participants.end()) {
        continue;
      }

      POI * other = *(participants.begin());
      if (other == poi) {
        other = *(++(participants.begin()));
      }
      if ((bg::get<0>(conflictInterval.first) >= bg::get<0>(visInterval.first)) && (bg::get<0>(conflictInterval.first) <= bg::get<0>(visInterval.second))) {
        conflictStarts.insert({bg::get<0>(conflictInterval.first), other});
      }
      if ((bg::get<0>(conflictInterval.second) >= bg::get<0>(visInterval.first)) && (bg::get<0>(conflictInterval.second) <= bg::get<0>(visInterval.second))) {
        conflictEnds.insert({bg::get<0>(conflictInterval.second), other});
      }
    }

    std::set<vertex_t> this_presence_set;

    /* Step 2: Create vertices representing possible selections */
    if (mtype == Heuristic::AM1) {
      if ((bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first)) >= MINIMUM_SELECTION_LENGTH) {
        vertex_t v = boost::add_vertex(*g);
        (*g)[v].weight = (bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first));
        (*g)[v].interval = visInterval;
        (*g)[v].poi = poi;
        (*g)[v].origVisibility = visInterval;
        (*g)[v].compound_weight = (bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first));
        (*g)[v].minimal = true;

#ifdef ENABLE_DEBUG
        if (poi->getId() == DBG_POI_1) {
          std::cout << "Adding vertex for POI " << poi->getLabel() << ": " << bg::get<0>(visInterval.first) << " -> " << bg::get<0>(visInterval.second) << "\n";
        }
#endif

        representants[poi->getInternalId()][{bg::get<0>(visInterval.first),bg::get<0>(visInterval.second)}] = v;
        this_presence_set.insert(v);
      }
    } else if (mtype == Heuristic::AM2) {
      /* A *start* of a conflict might be the *end* of a selected presence... */
      for (auto end : conflictStarts) {
        double endDist = end.first;
        if (endDist == bg::get<0>(visInterval.first))
          continue;

        if ((endDist  - bg::get<0>(visInterval.first)) >= MINIMUM_SELECTION_LENGTH) {
          // Already got that vertex
          if (representants[poi->getInternalId()].find({bg::get<0>(visInterval.first), endDist}) != representants[poi->getInternalId()].end())
            continue;

          vertex_t v = boost::add_vertex(*g);
          this_presence_set.insert(v);
          //std::cout << "Added vertex " << v << ": " << poi->getLabel() << " from " << bg::get<0>(visInterval.first) << "->" << endDist << "\n";

          (*g)[v].weight = (endDist  - bg::get<0>(visInterval.first));
          (*g)[v].interval = make_interval(bg::get<0>(visInterval.first), endDist);
          (*g)[v].poi = poi;
          (*g)[v].origVisibility = visInterval;

          representants[poi->getInternalId()][{bg::get<0>(visInterval.first), endDist}] = v;
        }
      }
      if (representants[poi->getInternalId()].find({bg::get<0>(visInterval.first),bg::get<0>(visInterval.second)}) == representants[poi->getInternalId()].end()) {
        if ((bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first)) >= MINIMUM_SELECTION_LENGTH) {
          vertex_t v = boost::add_vertex(*g);
          //std::cout << "Added vertex " << v << ": " << poi->getLabel() << " from " << bg::get<0>(visInterval.first) << "->" << bg::get<0>(visInterval.second) << "\n";

          this_presence_set.insert(v);
          (*g)[v].weight = (bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first));
          (*g)[v].interval = visInterval;
          (*g)[v].poi = poi;
          (*g)[v].origVisibility = visInterval;

          representants[poi->getInternalId()][{bg::get<0>(visInterval.first),bg::get<0>(visInterval.second)}] = v;
        }
      }
    } else if (mtype == Heuristic::AM3) {
      for (auto end : conflictStarts) {
        for (auto start : conflictEnds) {
          double endDist = end.first;
          double startDist = start.first;
          if (endDist <= startDist)
            continue;

          if ((endDist  - startDist) >= MINIMUM_SELECTION_LENGTH) {
            // Already got that vertex
            if (representants[poi->getInternalId()].find({startDist, endDist}) != representants[poi->getInternalId()].end())
              continue;


            vertex_t v = boost::add_vertex(*g);
            this_presence_set.insert(v);
            (*g)[v].weight = (endDist  - startDist);
            (*g)[v].interval = make_interval(startDist, endDist);
            (*g)[v].poi = poi;
            (*g)[v].origVisibility = visInterval;

            representants[poi->getInternalId()][{startDist, endDist}] = v;
          }
        }
      }
      for (auto end : conflictStarts) {
        double endDist = end.first;
        if (endDist == bg::get<0>(visInterval.first))
          continue;

        if ((endDist  - bg::get<0>(visInterval.first)) >= MINIMUM_SELECTION_LENGTH) {
          // Already got that vertex
          if (representants[poi->getInternalId()].find({bg::get<0>(visInterval.first), endDist}) != representants[poi->getInternalId()].end())
            continue;

          vertex_t v = boost::add_vertex(*g);
          this_presence_set.insert(v);
          (*g)[v].weight = (endDist  - bg::get<0>(visInterval.first));
          (*g)[v].interval = make_interval(bg::get<0>(visInterval.first), endDist);
          (*g)[v].poi = poi;
          (*g)[v].origVisibility = visInterval;

          representants[poi->getInternalId()][{bg::get<0>(visInterval.first), endDist}] = v;
        }
      }
      for (auto start : conflictEnds) {
        double startDist = start.first;
        if (startDist >= bg::get<0>(visInterval.second))
          continue;

        if ((bg::get<0>(visInterval.second) - startDist) >= MINIMUM_SELECTION_LENGTH) {
          // Already got that vertex
          if (representants[poi->getInternalId()].find({startDist, bg::get<0>(visInterval.second)}) != representants[poi->getInternalId()].end())
            continue;

          vertex_t v = boost::add_vertex(*g);
          this_presence_set.insert(v);
          (*g)[v].weight = (bg::get<0>(visInterval.second) - startDist);
          (*g)[v].interval = make_interval(startDist, bg::get<0>(visInterval.second));
          (*g)[v].poi = poi;
          (*g)[v].origVisibility = visInterval;

          representants[poi->getInternalId()][{startDist, bg::get<0>(visInterval.second)}] = v;
        }
      }
      if ((bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first)) >= MINIMUM_SELECTION_LENGTH) {
        if (representants[poi->getInternalId()].find({bg::get<0>(visInterval.first),bg::get<0>(visInterval.second)}) == representants[poi->getInternalId()].end()) {
          vertex_t v = boost::add_vertex(*g);
          this_presence_set.insert(v);
          (*g)[v].weight = (bg::get<0>(visInterval.second) - bg::get<0>(visInterval.first));
          (*g)[v].interval = visInterval;
          (*g)[v].poi = poi;
          (*g)[v].origVisibility = visInterval;

          representants[poi->getInternalId()][{bg::get<0>(visInterval.first),bg::get<0>(visInterval.second)}] = v;
        }
      }
    } else {
      throw "Unsupported model type";
    }

    if (boost::num_vertices(*g) > MAX_VERTICES) {
      delete g;
      return nullptr;
    }

    presence_sets.insert(this_presence_set);
  }

  /* Step 3: Link vertices representing the same presence */
  for (auto presence_set : presence_sets) {
    edges_total += presence_set.size() * presence_set.size() / 2;
    if (edges_total > MAX_EDGES) {
      delete g;
      return nullptr;
    }
    for (auto v1 : presence_set) {
      for (auto v2: presence_set) {
        if (v1 > v2) {
          boost::add_edge(v1, v2, {false, std::vector<Interval>()}, *g);
        }
      }
    }
  }

  /* Step 4: Link vertices representing the different, conflicting presences
   * by finding a triple-overlap of visibility1, visibility2 and a conflict */
  for (auto conflict : conflicts.queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    Interval conflictInterval = conflict.first;
    double conflictStart = bg::get<0>(conflictInterval.first);
    double conflictEnd = bg::get<0>(conflictInterval.second);
    std::set<POI *> participants = conflict.second;

    //std::cout << "Conflict between: " << (*(participants.begin()))->getLabel() << "(" <<(*(participants.begin()))->getId() << ")" << " and " << (*(++(participants.begin())))->getLabel() << "(" << (*(++(participants.begin())))->getId() << ")" << "\n";

    bool dbg_enable = false;
    if (dbg_set_equals(participants, DBG_POI_1, DBG_POI_2)) {
      //std::cout << "======== Conflict of DBG POIs\n";
      dbg_enable = true;
      //std::cout << "Conflict is: " << conflictStart << " -> " << conflictEnd << "\n";
      //std::cout << "------\n";
    }

    std::set<std::pair<std::pair<double, double>, vertex_t>> affected_1;
    for (auto repr_entry : representants[(*(participants.begin()))->getInternalId()]) {
      double start,end;
      std::tie(start,end) = repr_entry.first;
      vertex_t repr = repr_entry.second;

      if ((start <= conflictEnd) && (end >= conflictStart)) {
        affected_1.insert({{start, end}, repr});
      }
    }



    std::set<std::pair<std::pair<double, double>, vertex_t>> affected_2;
    for (auto repr_entry : representants[(*((++participants.begin())))->getInternalId()]) {
      double start,end;
      boost::tie(start,end) = repr_entry.first;
      vertex_t repr = repr_entry.second;

      if ((start <= conflictEnd) && (end >= conflictStart)) {
        affected_2.insert({{start, end}, repr});
      }
    }

    for (auto a1 : affected_1) {
      double start1, end1;
      boost::tie(start1, end1) = a1.first;
      vertex_t v1 = a1.second;
      for (auto a2 : affected_2) {
        double start2, end2;
        boost::tie(start2, end2) = a2.first;
        vertex_t v2 = a2.second;

        if ((start1 <= end2) && (end1 >= start2)) {

          conflict_edges.push_back(std::make_pair(std::set<ExpandedConflictGraph::vertex_t>({v1,v2}), conflictInterval));
          edges_total++;

          if (edges_total > MAX_EDGES) {
            delete g;
            return nullptr;
          }
        }
      }
    }

  }

  std::sort(conflict_edges.begin(), conflict_edges.end(), [&](const std::pair<std::set<vertex_t>, Interval> &e1, const std::pair<std::set<vertex_t>, Interval> &e2) {
    return e1.first < e2.first;
  });

  for (size_t i = 0 ; i < conflict_edges.size() ; ) {
    std::set<vertex_t> vertices = conflict_edges[i].first;
    std::vector<Interval> intervals;

    while ((i < conflict_edges.size()) && (conflict_edges[i].first == vertices)) {
      intervals.push_back(conflict_edges[i].second);
      i++;
    }

    //std::pair<bool, std::vector<Interval>> data = e.second;
    std::sort(intervals.begin(), intervals.end(), compareIntervals);
    //std::cout << "Adding edge between " << (*(vertices.begin())) << " and " << *(++(vertices.begin())) << "\n";
    boost::add_edge(*(vertices.begin()), *(++(vertices.begin())), {true, intervals}, *g);
  }


  if (!compound) {
    auto vertices = boost::vertices(*g);
    for (auto vit = vertices.first ; vit != vertices.second ; ++vit) {
      (*g)[*vit].compound_weight = (*g)[*vit].weight;
    }
  }

  return g;
}

void
ExpandedConflictGraph::writeToFile(const char *fileName, const ExpandedConflictGraph &g)
{
  std::ofstream outfile;
  outfile.open(fileName, std::ios::out | std::ios::trunc );

  boost::dynamic_properties dp;

  /* The built-in property maps have trouble casting stuff to strings. Oh well... */
  std::map<ExpandedConflictGraph::vertex_t, std::string> weightMap;
  auto vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    ExpandedConflictGraph::vertex_t v = *vit;
    double weight = g[v].weight;
    std::ostringstream os;
    os.precision(5);
    os << weight;
    weightMap[v] = os.str();
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::vertex_t, std::string> > actualWeightMap(weightMap);
  dp.property("weight", actualWeightMap);

  std::map<ExpandedConflictGraph::vertex_t, std::string> visibilityMap;
  vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    ExpandedConflictGraph::vertex_t v = *vit;
    Interval i = g[v].interval;
    std::ostringstream os;
    os << "[" << boost::geometry::get<0>(i.first) << " / " << boost::geometry::get<0>(i.second) << "]";
    visibilityMap[v] = os.str();
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::vertex_t, std::string> > actualVisibilityMap(visibilityMap);

  dp.property("visibility", actualVisibilityMap);

  std::map<ExpandedConflictGraph::vertex_t, std::string> origVisibilityMap;
  vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    ExpandedConflictGraph::vertex_t v = *vit;
    Interval i = g[v].origVisibility;
    std::ostringstream os;
    os << "[" << boost::geometry::get<0>(i.first) << " / " << boost::geometry::get<0>(i.second) << "]";
    origVisibilityMap[v] = os.str();
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::vertex_t, std::string> > actualOrigVisibilityMap(origVisibilityMap);

  dp.property("origVisibility", actualOrigVisibilityMap);

  std::map<ExpandedConflictGraph::vertex_t, std::string> labelMap;
  vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    ExpandedConflictGraph::vertex_t v = *vit;
    POI *poi = g[v].poi;
    std::ostringstream os;
    labelMap[v] = poi->getLabel();
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::vertex_t, std::string> > actualLabelMap(labelMap);
  dp.property("label", actualLabelMap);


  std::map<ExpandedConflictGraph::vertex_t, std::string> idMap;
  vs = boost::vertices(g);
  for (auto vit = vs.first; vit != vs.second; ++vit) {
    ExpandedConflictGraph::vertex_t v = *vit;
    POI *poi = g[v].poi;
    std::ostringstream os;
    os << poi->getId();
    idMap[v] = os.str();
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::vertex_t, std::string> > actualIdMap(idMap);
  dp.property("id", actualIdMap);

  std::map<ExpandedConflictGraph::edge_t, std::string> isConflictEdgeMap;
  auto es = boost::edges(g);
  for (auto eit = es.first; eit != es.second; ++eit) {
    ExpandedConflictGraph::edge_t e = *eit;
    bool val = g[e].conflict_edge;
    if (val) {
      isConflictEdgeMap[e] = "true";
    } else {
      isConflictEdgeMap[e] = "false";
    }
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::edge_t, std::string> > actualIsConflictEdgeMap(isConflictEdgeMap);
  dp.property("is_conflict", actualIsConflictEdgeMap);

  std::map<ExpandedConflictGraph::edge_t, std::string> conflictIntervalMap;
  es = boost::edges(g);
  for (auto eit = es.first; eit != es.second; ++eit) {
    ExpandedConflictGraph::edge_t e = *eit;
    std::vector<Interval> vals = g[e].intervals;
    std::ostringstream os;
    bool first = true;
    for (Interval val : vals) {
      if (!first) {
        os << ", ";
      } else {
        first = false;
      }
      os << "[" << boost::geometry::get<0>(val.first) << " / " << boost::geometry::get<0>(val.second) << "]";
    }
    conflictIntervalMap[e] = os.str();
  }
  boost::associative_property_map< std::map<ExpandedConflictGraph::edge_t, std::string> > actualconflictIntervalMap(conflictIntervalMap);
  dp.property("conflict_interval", actualconflictIntervalMap);

  boost::write_graphml(outfile, g, dp, false);

  outfile.close();
}
