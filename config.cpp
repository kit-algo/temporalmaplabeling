#include "config.h"

int num_threads = 1;

const QFont LABEL_FONT("Helvetica", 16);

double roadstyle_speeds[ROAD_STYLE_COUNT - 1] = {
  20, 50, 70
};
road_style roadstyle_styles[ROAD_STYLE_COUNT] = {
  {3.4, 3.0, QColor(87, 87, 87, 255), QColor(255,241,174, 255)},
  {4.4, 4.0, QColor(87, 87, 87, 255), QColor(255,230,115, 255)},
  {5.4, 5.0, QColor(87, 87, 87, 255), QColor(255,191,99, 255)},
  {6.4, 6.0, QColor(87, 87, 87, 255), QColor(255,171,51, 255)},
};

QColor poi_box_background(255,255,255,120);
QColor poi_box_outline(0,0,0,255);
double poi_box_dx = -10;
double poi_box_dy = -10;
double poi_box_dw = 30;

int TIME_LIMIT = 600;

/* Determines which OpenStreetMap POIs are used for labels. This is ORed. Thus, with these
   settings, any OSM node with "amenity: parking", or "amenity: fuel", …, or "tourism: hotel",…
   is considered. However, nodes which have no name set are discarded, as it is not possible
   to compute a useful label for them.
   */  
std::map<std::string, std::set<std::string>> POI_DEFAULT = {
    {"amenity", {"parking", "fuel", "atm", "restaurant", "cafe"}},
    {"tourism", {"hotel", "information", "motel"}},
    {"place", {"city", "suburb", "quarter", "neighborhood", "town", "village", "hamlet"}}
};
