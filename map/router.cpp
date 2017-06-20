#include "router.h"

#include <iostream>
#include <QDebug>

#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>

Router::Router(Map *map, int seed):
    map(map)
{
    this->rng = std::mt19937(seed);
}

std::vector<std::pair<edge_t, bool>> Router::get_random_route()
{
   vertex_t start = this->get_random_vertex();
   vertex_t end = start;

   while (start == end) {
       end = this->get_random_vertex();
   }

   //start = 3490;
   //end = 22040;

   //start = 20513;
   //end = 18336;

   std::vector<std::pair<edge_t, bool>> route = this->get_route(start, end);

   return route;
}

vertex_t Router::get_random_vertex()
{

    boost::graph_traits<Graph>::vertex_iterator vi;
    boost::graph_traits<Graph>::vertex_iterator vi_end;

    std::tie(vi, vi_end) = boost::vertices(this->map->get_graph());

    int max = boost::num_vertices(this->map->get_graph()) - 1;
    assert(max > 0);

    std::uniform_int_distribution<int> uni(0, max);
    int rand_index = uni(this->rng);

    // Since we only have the iterator interface, this is the only way
    // to access one at a random position
    return *(vi += rand_index);
}

std::vector<std::pair<edge_t, bool>> Router::get_route(vertex_t start, vertex_t end)
{
    typedef boost::property_map < Graph, boost::vertex_index_t >::type VertexIndexMap;
    typedef boost::property_map < Graph, boost::edge_index_t >::type EdgeIndexMap;

    typedef boost::iterator_property_map < vertex_t*, VertexIndexMap, vertex_t, vertex_t& > PredecessorMap;
    typedef boost::iterator_property_map < double*, VertexIndexMap, double, double& > DistanceMap;

    typedef boost::iterator_property_map < double*, EdgeIndexMap, double, double& > LengthMap;

    std::vector<vertex_t> predecessors(boost::num_vertices(this->map->get_graph())); // To store parents
    std::vector<double> distances(boost::num_vertices(this->map->get_graph()));

    VertexIndexMap vertexIndexMap = boost::get(boost::vertex_index, this->map->get_graph());
    PredecessorMap predecessorMap(&predecessors[0], vertexIndexMap);
    DistanceMap distanceMap(&distances[0], vertexIndexMap);

    std::vector<double> lengths(boost::num_edges(this->map->get_graph()));
    EdgeIndexMap edgeIndexMap = boost::get(boost::edge_index, this->map->get_graph());
    LengthMap lengthMap(&lengths[0], edgeIndexMap);

    // Populate the length
    boost::graph_traits<Graph>::edge_iterator ei;
    boost::graph_traits<Graph>::edge_iterator ei_end;
    boost::tie(ei, ei_end) = boost::edges(this->map->get_graph());

    for (; ei != ei_end; ei++) {
        edge_t edge = *ei;
        boost::put(lengthMap, edge, this->map->get_way_length(edge));
    }

    boost::dijkstra_shortest_paths(this->map->get_graph(), start, boost::distance_map(distanceMap).predecessor_map(predecessorMap).weight_map(lengthMap));

    std::vector<std::pair<edge_t, bool>> path;

    vertex_t v = end;
    for(vertex_t u = predecessorMap[v];
        u != v;
        v = u, u = predecessorMap[v])
    {
      std::pair<boost::graph_traits<Graph>::edge_descriptor, bool> edgePair = boost::edge(u, v, this->map->get_graph());
      Graph::edge_descriptor edge = edgePair.first;

      if (u > v) {
          path.push_back(std::make_pair(edge, true));
      } else {
          path.push_back(std::make_pair(edge, false));
      }
    }

    assert(path.size() > 0);

    return path;
}
