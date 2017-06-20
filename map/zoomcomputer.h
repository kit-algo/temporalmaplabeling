#ifndef ZOOMCOMPUTER_H
#define ZOOMCOMPUTER_H

#include "sizecomputer.h"
#include "trajectory.h"
#include "map.h"

#include <CGAL/Cartesian.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
typedef CGAL::Quotient<CGAL::MP_Float> Number_type;
typedef CGAL::Cartesian<Number_type> Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef Traits_2::Point_2 Point_2;
typedef Traits_2::X_monotone_curve_2 Segment_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;


class ZoomComputer
{
public:
  ZoomComputer(Trajectory *t, double w, double h, Map *map);
  void run();

private:
  Trajectory *t;
  Arrangement_2 arr;
  SizeComputer scomp;

  double end_x;

  void make_basic_segments();
  void make_backwards_segments();
  void make_forwards_segments();

  void apply_zoom();

  double interpolate_y_at_x_on_curve(Arrangement_2::Halfedge seg, double x);

  void dbg_print_hole(Arrangement_2::Ccb_halfedge_circulator circ);
  double dbg_items_length(std::vector<TrajectoryItem *> items);

  std::map<TrajectoryItem *, Point_2> item_starts;
  std::map<TrajectoryItem *, Point_2> item_ends;
};

#endif
