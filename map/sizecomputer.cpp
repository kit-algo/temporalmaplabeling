#include "sizecomputer.h"

#include "map/map.h"
#include "config.h"

SizeComputer::SizeComputer(double w, double h, Map *map)
: screen_width(w), screen_height(h)
{
  double median_lat = map->get_median_latitude();
  Position pos1 = qMakePair(0.0, median_lat);
  Position pos2 = qMakePair(1.0, median_lat);

  this->map_unit_dist = Map::compute_distance(pos1, pos2);
}

double
SizeComputer::zoom_for_meter_per_pixel(double mpp)
{
  return (this->map_unit_dist/mpp);
}

double
SizeComputer::zoom_for_speed(double speed) {
  return this->zoom_for_meter_per_pixel(this->mpp_for_speed(speed));
}

double
SizeComputer::seconds_to_meters_at_speed(double speed, double seconds)
{
  return speed / 3.6 * seconds;
}

double
SizeComputer::mpp_for_speed(double speed) {
  double meter_per_second = speed / 3.6;

  return meter_per_second * TIME_VISIBLE_SECONDS / (std::min(CAMERA_WIDTH, CAMERA_HEIGHT));
}
