#ifndef MAPREADER_H
#define MAPREADER_H

#include <string>
#include <map>
#include <utility>

#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/types.hpp>

#include "map/map.h"
#include "map/projection.h"

using namespace std;

/* Helper class that reads in an OSM file and turns it into a Map object.
 */
class MapReader
{
public:
    MapReader(const string osmFile, const string wayFile);
    ~MapReader();

    void run();
    Map& get_map();

private:
    const string osmFile;
    const string wayFile;

    Map map;
    MercatorProjection proj;

    void readOSM();
    void readWays();

    /* Parses nodes and adds them to the map if a node constitutes a POI. If a node
     * is just on a way, it will not be added to the map.
     */
    void parseOSMNode(osmium::Node &node);

    /* Parses the ways, splitting them up as necessary. This does not yet add anything
     * to the map, since every way parsed might be split later by another way intersecting
     * it.
     */
    void parseWay(osmium::Way &way);

    /* Commits the parsed ways to the map, after all ways were parsed.
     */
    void commitWays();

    void commitPOIs();

    // only valid during run()
    std::map<osmium::object_id_type, Position> node_to_pos;
    std::vector<POI> preliminaryPOIs;
    std::set<Position> positionsSeen;

    // Holds all the edges
    // Format is: pair of node ids -> maximum speed on that edge
    std::map<std::pair<int, int>, double> edges;
};

#endif // MAPREADER_H
