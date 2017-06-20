#include <map>
#include <utility>

#include "map/mapreader.h"
#include "map/map.h"

#include <QDebug>
#include <fstream>
#include <sstream>


#include <osmium/io/xml_input.hpp>


MapReader::MapReader(string osmFile, string wayFile):
    osmFile(osmFile), wayFile(wayFile)
{
}

MapReader::~MapReader()
{
}

void
MapReader::readOSM()
{
  osmium::io::Reader reader(this->osmFile);
  osmium::io::Header header = reader.header();

  this->node_to_pos.clear();
  this->edges.clear();

  while (osmium::memory::Buffer buffer = reader.read()) {
      for (auto& entity : buffer) {
          switch (entity.type())
          {
          case osmium::item_type::node:
              this->parseOSMNode(static_cast<osmium::Node&>(entity));
              break;
          default:
              // We don't need anything else.
              break;
          }
      }
  }
}

void
MapReader::readWays()
{
  std::ifstream infile(this->wayFile);
  std::string line;

  // Read Header
  line = "#foo";
  while (line[0] == '#') {
    std::getline(infile, line);
  }
  int nodecount = atoi(line.c_str());
  std::getline(infile, line);
  while (line[0] == '#') {
    std::getline(infile, line);
  }
  int edgecount = atoi(line.c_str());

  /* Read Nodes and their positions */
  for (int i = 0 ; i < nodecount ; i++) {
    std::getline(infile, line);
    while (line[0] == '#') {
      std::getline(infile, line);
    }

    std::istringstream iss(line);
    int id;
    double lat, lon;
    iss >> id >> lat >> lon;

    double x = lon;
    double y = lat;

    this->proj.project(&x, &y);

    this->node_to_pos.insert(std::make_pair(id, qMakePair(x, y)));
    this->positionsSeen.insert(qMakePair(x, y));
  }

  /* Read edges between nodes */
  for (int i = 0 ; i < edgecount ; i++) {
    std::getline(infile, line);
    while (line[0] == '#') {
      std::getline(infile, line);
    }

    std::istringstream iss(line);
    int source_id, target_id;
    double length, speed;
    string dummy1, dummy2, dummy3;
    iss >> source_id >> target_id >> length >> dummy1 >> speed >> dummy2 >> dummy3;

    this->edges.insert(std::make_pair(std::make_pair(source_id, target_id), speed));
  }
}

void
MapReader::run()
{
  this->readOSM();
  this->readWays();

  this->commitWays();
  this->commitPOIs();

  // deconstruct to save memory
  this->node_to_pos.clear();
  this->edges.clear();
  this->preliminaryPOIs.clear();
  this->positionsSeen.clear();
}

Map &MapReader::get_map()
{
    return this->map;
}

void MapReader::parseOSMNode(osmium::Node &node)
{
    /* In the node_ref_list that will be available in the ways later, all locations are invalid.
     * Not sure if that is a bug or limitation, but that makes it necessary to store the locations
     * now. */

     if (node.tags().get_value_by_key("name") == nullptr) {
         // Things without names are no good
         return;
     }
    double x = (double)node.location().lon();
    double y = (double)node.location().lat();

    this->proj.project(&x, &y);

    std::map<std::string, std::string> classes;
    for (auto key : POI::CLASS_KEYS) {
        if (node.tags().get_value_by_key(key.c_str()) != nullptr) {
            classes.insert(std::make_pair(key, node.tags().get_value_by_key(key.c_str())));
        }
    }

    if (classes.size() == 0) {
        // No class key? Can't be selected!
        return;
    }


    if (node.tags().get_value_by_key("amenity") != nullptr) {
        string name = node.tags().get_value_by_key("name");
        QPair<double, double> pos = qMakePair(x, y);
        POI poi(name, pos, classes, node.id());
        this->preliminaryPOIs.push_back(poi);
    }
}

void MapReader::commitWays()
{
  for (auto edge : this->edges) {
    int source = edge.first.first;
    int target = edge.first.second;
    double speed = edge.second;

    double sx = this->node_to_pos[source].first;
    double sy = this->node_to_pos[source].second;
    double tx = this->node_to_pos[target].first;
    double ty = this->node_to_pos[target].second;

    std::vector<Position> positions;
    positions.push_back(qMakePair(sx,sy));
    positions.push_back(qMakePair(tx,ty));
    this->map.add_way(std::move(positions), speed);
  }
}

void MapReader::commitPOIs()
{
    for (auto prePOI : this->preliminaryPOIs) {
        double x = prePOI.getPos().first;
        double y = prePOI.getPos().second;

        POI *poi = new POI(prePOI.getLabel(), qMakePair(x,y), prePOI.getClasses(), prePOI.getId());
        this->map.add_poi(poi);
    }
}
