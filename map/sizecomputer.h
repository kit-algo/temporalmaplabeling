#ifndef SIZECOMPUTER_H
#define SIZECOMPUTER_H

#include "map/map.h"

class SizeComputer {
public:
  SizeComputer(double w, double h, Map *map);

  double zoom_for_speed(double speed);
  double zoom_for_meter_per_pixel(double mpp);

  double seconds_to_meters_at_speed(double speed, double seconds);

private:
  double screen_width;
  double screen_height;
  double map_unit_dist;

  double mpp_for_speed(double speed);
};

#endif
