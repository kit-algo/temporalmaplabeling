#include "map/map.h"

#include "projection.h"

#include <QDebug>
#include <QString>

#include <tuple>
#include <math.h>

#include "projection.h"

MercatorProjection Map::mercator_projection;

long POI::MAX_ID = 0;

Map::Map():
    G()
{
  this->median_lat = std::nan("");
}

Map::~Map()
{
    for (POI *poi : this->poi) {
        delete poi;
    }
}

double
Map::get_median_latitude()
{
  if (std::isnan(this->median_lat)) {
    std::vector<double> latitudes;

    for (auto poi : this->poi) {
      latitudes.push_back(poi->getPos().second);
    }
    std::sort(latitudes.begin(), latitudes.end());

    this->median_lat = latitudes[latitudes.size() / 2];
  }

  return this->median_lat;
}

void Map::add_poi(POI *poi)
{
    this->poi.push_back(poi);
    this->poiTree.insert(std::make_pair(this->posToPoint(poi->getPos()), poi));

    if (poi->matchFilter(this->poiFilter)) {
        this->selectedPOI.insert(poi);
    }

    this->median_lat = std::nan("");
}

double
Map::get_way_speed(edge_t edge)
{
  return boost::get(boost::edge_speed, this->G)[edge];
}

void Map::add_way(std::vector<Position> positions, double speed)
{
    Position start = positions.front();
    Position end = positions.back();

    if (start == end) {
        // Closed ways are not interesting
        return;
    }


    vertex_t start_vertex;
    auto it = this->pos_to_vertex.find(start);
    if (it != this->pos_to_vertex.end()) {
        start_vertex = it->second;
    } else {
        start_vertex = boost::add_vertex(this->G);
        this->pos_to_vertex[start] = start_vertex;
        this->vertex_to_pos[start_vertex] = start;
    }

    vertex_t end_vertex;
    it = this->pos_to_vertex.find(end);
    if (it != this->pos_to_vertex.end()) {
        end_vertex = it->second;
    } else {
        end_vertex = boost::add_vertex(this->G);
        this->pos_to_vertex[end] = end_vertex;
        this->vertex_to_pos[end_vertex] = end;
    }

    edge_t way_edge;
    std::tie(way_edge, std::ignore) = boost::add_edge(start_vertex, end_vertex, EdgeProperties(0.0, EdgeSpeed(speed)), this->G);

    this->ways[way_edge] = std::move(positions);
    if (start_vertex > end_vertex) {
        // Waypoints are ordered from smaller to larger vertices, so that we can later piece
        // together the strings of waypoints.
        std::reverse(this->ways[way_edge].begin(), this->ways[way_edge].end());
    }

    this->way_length[way_edge] = this->compute_length(this->ways[way_edge]);
}

Position Map::get_vertex_pos(vertex_t v)
{
    return this->vertex_to_pos[v];
}

double Map::get_way_length(edge_t edge)
{
    return this->way_length[edge];
}

Graph &Map::get_graph()
{
    return this->G;
}

std::vector<Position> Map::get_waypoints(edge_t edge)
{
    return this->ways[edge];
}

Point Map::posToPoint(Position pos)
{
    return Point(pos.first, pos.second);
}

double Map::compute_length(std::vector<Position> positions)
{
    double total = 0.0;

    Position last = positions[0];
    for (Position pos : positions) {
        if (pos == last) {
            continue;
        }

        //std::cout << "Distance from " << last.first << "/" << last.second << " to " << pos.first << "/" << pos.second << ": " << Map::compute_distance(last, pos) << "\n";
        total += Map::compute_distance(last, pos);
        last = pos;
    }

    return total;
}

void Map::set_poi_filter(std::map<std::string, std::set<std::string> > filter)
{
    this->selectedPOI.clear();
    this->poiFilter = filter;
    this->poiTree.clear();

    for (unsigned int i  = 0 ; i < this->poi.size(); i++) {
        POI *poi = this->poi[i];
        if (poi->matchFilter(filter)) {
            this->selectedPOI.insert(poi);
            this->poiTree.insert(std::make_pair(this->posToPoint(poi->getPos()), poi));
        }
    }
}

// Taken from
// https://stackoverflow.com/questions/365826/calculate-distance-between-2-gps-coordinates
double Map::compute_distance(Position pos1, Position pos2)
{
  double x1 = pos1.first;
  double y1 = pos1.second;
  mercator_projection.reverseProject(&x1, &y1);

  double x2 = pos2.first;
  double y2 = pos2.second;
  mercator_projection.reverseProject(&x2, &y2);

  double R = 6371000; // m
  double dLatDegree = (y1 - y2);
  double dLonDegree = (x1 - x2);
  double dLat = dLatDegree * M_PI / 180.0;
  double dLon = dLonDegree * M_PI / 180.0;
  double lat1 = y1 * M_PI / 180.0;
  double lat2 = y2 * M_PI / 180.0;

  double a = sin(dLat/2.0) * sin(dLat/2.0) + sin(dLon/2.0) * sin(dLon/2.0) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1.0-a));
  return R * c;
}

double Map::compute_distance_latlon(Position pos1, Position pos2)
{
    double dx = pos1.first - pos2.first;
    double dy = pos1.second - pos2.second;
    return sqrt(dx * dx + dy * dy);
}

const std::set<POI *> &Map::getPOI() const
{
    return this->selectedPOI;
}

const std::set<POI *> Map::getPOIsWithin(Position leftLower, Position rightUpper)
{
    std::vector<std::pair<Point, POI *>> indices;
    Box box(this->posToPoint(leftLower), this->posToPoint(rightUpper));
    this->poiTree.query(boost::geometry::index::within(box), std::back_inserter(indices));

    std::set<POI *> visible;
    for (auto index : indices) {
        visible.insert(index.second);
    }

    return visible;
}


POI::POI(const std::string &label, Position pos, std::map<std::string, std::string> classes, osmium::object_id_type id):
    label(label), pos(pos), id(id), classes(classes)
{
  this->internal_id = POI::MAX_ID++;
}

long POI::getInternalId() const {
  return this->internal_id;
}

long POI::getMaxIntenalId() {
  return POI::MAX_ID;
}

Position POI::getPos()
{
    return this->pos;
}

const std::string &POI::getLabel()
{
    return this->label;
}

std::string POI::getClass(std::string key) const
{
    if (this->classes.find(key) != this->classes.end()) {
        return this->classes.at(key);
    } else {
        return "";
    }
}

std::map<std::string, std::string> POI::getClasses() const
{
    return classes;
}

bool POI::matchFilter(std::map<std::string, std::set<std::string> > &filter)
{
    if (filter.size() == 0) {
        return true;
    }

    for (auto key : this->classes) {
        if ((filter.find(key.first) != filter.end()) && (filter[key.first].find(key.second) != filter[key.first].end())) {
            return true;
        }
    }

    return false;
}

osmium::object_id_type POI::getId() const
{
    return id;
}

// TODO factor out
const std::set<std::string> POI::CLASS_KEYS = { "amenity", "building", "emergency", "historic", "leisure", "tourism", "place" };
