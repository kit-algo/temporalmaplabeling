#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include <utility>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include <osmium/osm/types.hpp>

#include <QPair>

#include "position.h"
#include "projection.h"

/* The property that will hold the maximum speed on an edge */
namespace boost {
  enum edge_speed_t { edge_speed };
  template <> struct property_kind<edge_speed_t> {
    typedef boost::edge_property_tag type;
  };
  //BOOST_INSTALL_PROPERTY(boost::edge, speed);
}

/* This "Point" datatype is only used for storage in the fast-lookup rtree. In all other
 * instances, the "Position" datatype is used.
 */
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> Point;
typedef boost::geometry::model::box<Point> Box;

/* These are the graph datatypes used for reading the OSM data into. This graph is also used
 * for routing etc.
 */

typedef boost::property < boost::edge_speed_t, double,
                          boost::property < boost::edge_index_t, std::size_t>> EdgeSpeed;
typedef boost::property < boost::edge_weight_t, double,
                          EdgeSpeed > EdgeProperties;
typedef boost::adjacency_list<boost::vecS, boost::vecS,
                                boost::undirectedS,
                                boost::property < boost::vertex_name_t, std::string >,
                                EdgeProperties
                            > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;


/* A POI is any point on the map that we want to place a label on.
 */
class POI
{
public:
    /* Constructs the POI
     *
     * Arguments:
     *  label:      The string to label the POI with
     *  pos:        The position of the POI's anchor point
     *  classes:    A list of classes that this POI belongs to. Used for filtering.
     */
    POI(const std::string &label, Position pos, std::map<std::string, std::string> classes, osmium::object_id_type id);

    Position getPos();
    const std::string& getLabel();
    std::string getClass(std::string key) const;

    std::map<std::string, std::string> getClasses() const;
    bool matchFilter(std::map<std::string, std::set<std::string> > &filter);

    static const std::set<std::string> CLASS_KEYS;

    osmium::object_id_type getId() const;

    long getInternalId() const;
    static long getMaxIntenalId();
private:
    const std::string label;
    Position pos;
    osmium::object_id_type id;
    std::map<std::string, std::string> classes;

    long internal_id;
    static long MAX_ID;
};

/* The map is the main datastructure holding all information read from the OSM input. It knows
 * about ways and POIs on the map.
 */
class Map
{
public:
    Map();
    ~Map();

    void add_poi(POI *poi);
    void add_way(std::vector<Position> positions, double speed);

    Position get_vertex_pos(vertex_t v);

    /* Returns the length of the way associated with an edge of the graph. The length is
     * returned in meters!
     */
    double get_way_length(edge_t edge);

    /* Returns the maximum speed that can be driven on a given edge */
    double get_way_speed(edge_t edge);

    Graph& get_graph();
    std::vector<Position> get_waypoints(edge_t edge);

    /* Computes the distance between two points, in meters.
     */
    static double compute_distance(Position pos1, Position pos2);

    /* Computes the distance between two points, but in native lat/lon-coordinates,
     * i.e. without conversion to meters.
     */
    static double compute_distance_latlon(Position pos1, Position pos2);

    double get_median_latitude();

    const std::set<POI*>& getPOI() const;
    const std::set<POI*> getPOIsWithin(Position leftLower, Position rightUpper);
    void set_poi_filter(std::map<std::string, std::set<std::string>> filter);

private:
    Point posToPoint(Position pos);

    Graph G;
    std::map<Position, vertex_t> pos_to_vertex;
    std::map<edge_t, std::vector<Position>> ways;
    std::map<edge_t, double> way_length;
    std::map<vertex_t, Position> vertex_to_pos;

    double compute_length(std::vector<Position> positions);

    // Map is responsible for these. They are destroyed in the destructor.
    std::vector<POI*> poi;
    boost::geometry::index::rtree<std::pair<Point, POI *>, boost::geometry::index::quadratic<8>> poiTree;
    std::set<POI*> selectedPOI;
    std::map<std::string, std::set<std::string>> poiFilter;

    double median_lat;

    static MercatorProjection mercator_projection;
};

#endif // MAP_H
