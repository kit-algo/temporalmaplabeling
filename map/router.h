#ifndef ROUTER_H
#define ROUTER_H

#include <random>

#include "map/map.h"

class Router
{
public:
    Router(Map *map, int seed);

    /*
     * The return value is a list of pairs: every pair contains an
     * edge (which is the next edge traversed in the path, and a
     * boolean, which will be True if the edge is traversed in the
     * direction it has in the graph, and false otherwise.
     */
    std::vector<std::pair<edge_t, bool>> get_random_route();

private:
    Map* map;
    vertex_t get_random_vertex();
    std::vector<std::pair<edge_t, bool>> get_route(vertex_t start, vertex_t end);

    std::mt19937 rng;
};

#endif // ROUTER_H
