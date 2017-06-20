#include "zoomcomputer.h"
#include "config.h"

ZoomComputer::ZoomComputer(Trajectory *t, double w, double h, Map *map)
  : t(t), scomp(w,h, map)
{}

void ZoomComputer::run()
{
  this->make_basic_segments();
  this->make_backwards_segments();
  this->make_forwards_segments();

  this->apply_zoom();
}

void ZoomComputer::dbg_print_hole(Arrangement_2::Ccb_halfedge_circulator circ)
{
  Arrangement_2::Ccb_halfedge_const_circulator curr = circ;
  std::cout << "(" << curr->source()->point() << ")";
  do {
    std::cout << " [" << curr->curve() << "] "
              << "(" << curr->target()->point() << ")\n";
  } while (++curr != circ);
}

double ZoomComputer::interpolate_y_at_x_on_curve(Arrangement_2::Halfedge seg, double x)
{
  double x_start = CGAL::to_double(seg.source()->point().x());
  double y_start = CGAL::to_double(seg.source()->point().y());
  double x_end = CGAL::to_double(seg.target()->point().x());
  double y_end = CGAL::to_double(seg.target()->point().y());

  std::cout << "Interpolation: [" << x_start << " / " << y_start << " -> " << x_end << " / " << y_end << "], length " << (x_end - x_start) << ", part" << (x - x_start) << "\n";

  double res;
  if (std::abs(x_end - x_start) < 0.00001) {
    res = (y_start + y_end) / 2.0;
  } else {
    double xfrac = (x - x_start) / (x_end - x_start);
    res = y_start + ((y_end - y_start) * xfrac);
  }
  assert (res > (-1 * std::numeric_limits<double>::infinity()));
  return res;
}

double
ZoomComputer::dbg_items_length(std::vector<TrajectoryItem *> items)
{
#ifdef ENABLE_DEBUG
  double sum = 0.0;
  for (auto item : items) {
    sum += item->getLength();
  }
  return sum;
#else
  return 0.0;
#endif
}


#define BIGNUM 100.0
#define MINIMUM_SEG_LENGTH 0.001

void ZoomComputer::apply_zoom()
{
  Arrangement_2::Face_handle f = this->arr.unbounded_face();
  Arrangement_2::Hole_iterator hi = f->holes_begin();
  Arrangement_2::Ccb_halfedge_circulator outer = *hi;
  Arrangement_2::Ccb_halfedge_circulator outer_end;

  // See that the arrangement is connected
#ifdef ENABLE_DEBUG
  hi++;
  if (hi != f->holes_end()) {
    std::cout << "First hole: \n";
    this->dbg_print_hole(outer);
    std::cout << "Second hole: \n";
    this->dbg_print_hole(*hi);
  }
  assert(hi == f->holes_end());
#endif

  // Find the end
  while (outer->target()->point().x() != this->end_x) {
    outer++;
  }
  outer++;
  outer_end = outer;

  std::cout << "End segment is: " << CGAL::to_double(outer_end->source()->point().x()) << "/" << CGAL::to_double(outer_end->source()->point().y()) << " / " << CGAL::to_double(outer_end->target()->point().x()) << "/" << CGAL::to_double(outer_end->target()->point().y()) << "\n";

  // Fast forward to the start
  while ((CGAL::to_double(outer->source()->point().x()) != 0) || (CGAL::to_double(outer->target()->point().x()) <= 0.01)) {
    outer++;
  }

  Arrangement_2::Ccb_halfedge_circulator dbg_outer = outer;
  do {
    std::cout << CGAL::to_double(dbg_outer->source()->point().x()) << "/" << CGAL::to_double(dbg_outer->source()->point().y()) << " -> " <<   CGAL::to_double(dbg_outer->target()->point().x()) << "/" << CGAL::to_double(dbg_outer->target()->point().y()) << "\n";
    dbg_outer++;
  } while (dbg_outer != outer);


  // Actually build the zoom items
  double current_zoom = CGAL::to_double(outer->target()->point().y());
  double cur_x = 0.0;

  std::vector<TrajectoryItem*> new_items;
  for (TrajectoryItem *item : this->t->items) {
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      item->adjustZoomLevel(current_zoom);
      new_items.push_back(item);
      assert(item->getLength() < 1000000);
      continue;
    }
    StraightTrajectoryItem *sitem = (StraightTrajectoryItem *)item;
    double x_done = 0.0;

    std::cout << "=================\nSTI is now: ";
    sitem->dbg_print();
    std::cout << "Item from " << cur_x << " to " << (cur_x + item->getLength()) << " (length " << item->getLength() << ")\n";
    std::cout << "Start segment for this item: " << CGAL::to_double(outer->source()->point().x()) << " -> " << CGAL::to_double(outer->target()->point().x()) << "\n";


    while ((outer != outer_end) && (CGAL::to_double(outer->target()->point().x()) < (cur_x + item->getLength()))) {
      double new_zoom = CGAL::to_double(outer->target()->point().y());
      std::cout << "----> Current segment: " << CGAL::to_double(outer->source()->point().x()) << " / " << CGAL::to_double(outer->source()->point().y()) << " -> " << CGAL::to_double(outer->target()->point().x()) << " / " << CGAL::to_double(outer->target()->point().y()) << "\n";


      Position start;
      Position end;

      if (item->getLength() < MINIMUM_SEG_LENGTH) {
        start = end = sitem->getStart();
      } else {
        CarPosition startCarpos = sitem->interpolatePosition(x_done);
        start = startCarpos.pos;
        double we_are_here = CGAL::to_double(outer->target()->point().x());
        CarPosition endCarpos = sitem->interpolatePosition(we_are_here - cur_x);
        end = endCarpos.pos;
      }

      StraightTrajectoryItem *zoomItem = new StraightTrajectoryItem(start, end, current_zoom, new_zoom, sitem->getSpeed());
      new_items.push_back(zoomItem);
      assert(zoomItem->getLength() < 1000000);
      current_zoom = new_zoom;
      std::cout << "Adding: ";
      zoomItem->dbg_print();

      x_done = CGAL::to_double(outer->target()->point().x()) - cur_x;
      outer++;
      std::cout << "Advancing to: " << CGAL::to_double(outer->target()->point().x()) << "\n";
    }
    std::cout << "----> Segment after loop: " << CGAL::to_double(outer->source()->point().x()) << " / " << CGAL::to_double(outer->source()->point().y()) << " -> " << CGAL::to_double(outer->target()->point().x()) << " / " << CGAL::to_double(outer->target()->point().y()) << "\n";


    // Add the trailing remainder
    if (outer != outer_end) {
      Position start;
      Position end;
      if (item->getLength() < 0.01) {
        start = end = sitem->getStart();
      } else {
        CarPosition startCarpos = sitem->interpolatePosition(x_done);
        start = startCarpos.pos;
        CarPosition endCarpos = sitem->interpolatePosition(item->getLength());
        end = endCarpos.pos;
      }
      double final_zoom = this->interpolate_y_at_x_on_curve(*outer, (cur_x + item->getLength()));
      StraightTrajectoryItem *finalItem = new StraightTrajectoryItem(start, end, current_zoom, final_zoom, sitem->getSpeed());
      new_items.push_back(finalItem);
      std::cout << "Adding Final: ";
      finalItem->dbg_print();
#ifdef ENABLE_DEBUG
      if (finalItem->getLength() < MINIMUM_SEG_LENGTH) {
        assert(std::abs(current_zoom - final_zoom) < 0.001);
      }
#endif
      current_zoom = final_zoom;
    }

    if (item->getLength() > MINIMUM_SEG_LENGTH) {
      cur_x += item->getLength();
    }
  }

  Arrangement_2::Ccb_halfedge_circulator curr = outer;
  do {
    std::cout << " [" << curr->curve() << "] "
              << "(" << curr->source()->point().x() << ") \n";
  } while (++curr != outer);

  this->t->items = new_items;
}



void ZoomComputer::make_basic_segments()
{
  double cur_zoom = 0.0, last_zoom = 0.0;
  double cur_x = 0.0;
  std::vector<TrajectoryItem *> items = this->t->getItems();

  cur_zoom = this->scomp.zoom_for_speed(items[0]->getSpeed());

  double max_zoom = -1 * std::numeric_limits<double>::infinity();
  double min_zoom = -1.0;

  for (TrajectoryItem *item : items) {
    double desired_zoom = this->scomp.zoom_for_speed(item->getSpeed());
    max_zoom = std::max(max_zoom, desired_zoom);
    min_zoom = std::min(min_zoom, desired_zoom);
  }

  // add left dummy straight
  Point_2 left_dummy_s (0.0, max_zoom + BIGNUM), left_dummy_t (0.0, min_zoom - BIGNUM);
  Segment_2 left_dummy (left_dummy_s, left_dummy_t);

  std::cout << "Dummy start: 0.0/" << (max_zoom + BIGNUM) << " -> 0.0/" <<  (min_zoom - BIGNUM) << "\n";

  // TODO is it okay to pass a reference to an ephermal object here?
  CGAL::insert(this->arr, left_dummy);

  Point_2 last_point(0.0, this->scomp.zoom_for_speed(items[0]->getSpeed()));

  for (TrajectoryItem *item : items) {
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      continue;
    }

    if (item->getLength() == 0.0) {
      continue;
    }

    last_zoom = cur_zoom;
    cur_zoom = this->scomp.zoom_for_speed(item->getSpeed());
    std::cout << "Speed: " << item->getSpeed() << " / Desired Zoom level: " << cur_zoom << "\n";

    if (cur_zoom != last_zoom) {
      // Make a vertical segment to the start of this item
      Point_2 vert_t(cur_x, cur_zoom);
      std::cout << "~ VSeg: " << cur_x << "/" << last_zoom << " -> " << cur_x << "/" << cur_zoom << "\n";
      Segment_2 vert (last_point, vert_t);
      last_point = vert_t;
      CGAL::insert(this->arr, vert);
    }

    this->item_starts[item] = last_point;

    // Make horizontal segment
    if (item->getLength() > MINIMUM_SEG_LENGTH) {
      Point_2 hor_t(cur_x + item->getLength(), cur_zoom);
      Segment_2 hor (last_point, hor_t);
      CGAL::insert(this->arr, hor);
      std::cout << "~ HSeg: " << cur_x << "/" << cur_zoom << " -> " <<  cur_x + item->getLength() << "/" << cur_zoom << " (Length: " << item->getLength() << ")\n";
      last_point = hor_t;
      cur_x += item->getLength();
    }

    this->item_ends[item] = last_point;
  }

  // add right dummy straight
  Point_2 right_dummy_s (cur_x, max_zoom + BIGNUM), right_dummy_t (cur_x, min_zoom - BIGNUM);
  std::cout << "Dummy end: " << cur_x << "/" << (max_zoom + BIGNUM) << " -> " << cur_x << "/" <<  (min_zoom - BIGNUM) << "\n";
  Segment_2 right_dummy (right_dummy_s, right_dummy_t);
  CGAL::insert(this->arr, right_dummy);
  this->end_x = cur_x;
}


void ZoomComputer::make_forwards_segments()
{
  double cur_zoom = 0.0, last_zoom = 0.0;
  double cur_x = 0.0;
  std::vector<TrajectoryItem *> items = this->t->getItems();

  cur_zoom = this->scomp.zoom_for_speed(items[0]->getSpeed());
  TrajectoryItem *last_item = nullptr;

  for (TrajectoryItem *item : items) {
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      continue;
    }

    if (item->getLength() == 0.0) {
      continue;
    }

    last_zoom = cur_zoom;
    cur_zoom = this->scomp.zoom_for_speed(item->getSpeed());

    if (cur_zoom < last_zoom) {
      double x_at_zeropoint = cur_x + (last_zoom / ZOOM_PER_METER);
      // Make zoom-down segment
      Point_2 zd_t(x_at_zeropoint, 0.0);
      Segment_2 zd (this->item_ends[last_item], zd_t);
      std::cout << "~ DSeg: " << cur_x << "/" << last_zoom << " -> " << x_at_zeropoint << "/0.0\n";
      CGAL::insert(this->arr, zd);
    }

    if (item->getLength() > MINIMUM_SEG_LENGTH) {
      cur_x += item->getLength();
    }

    last_item = item;
  }
}

void ZoomComputer::make_backwards_segments()
{
  double cur_zoom = 0.0, last_zoom = 0.0;
  double cur_x = this->end_x;
  std::vector<TrajectoryItem *> items = this->t->getItems();

  cur_zoom = this->scomp.zoom_for_speed(items[items.size() - 1]->getSpeed());
  TrajectoryItem *last_item = items[items.size() - 1];

  for (int i = items.size() - 1 ; i >= 0 ; i--) {
    TrajectoryItem *item = items[i];
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      continue;
    }

    if (item->getLength() == 0.0) {
      continue;
    }

    last_zoom = cur_zoom;
    cur_zoom = this->scomp.zoom_for_speed(item->getSpeed());

    if (cur_zoom < last_zoom) {
      double x_at_zeropoint = cur_x - (last_zoom / ZOOM_PER_METER);
      // Make zoom-up segment
      Point_2 zu_t(x_at_zeropoint, 0.0);
      Segment_2 zu (this->item_starts[last_item], zu_t);
      std::cout << "~ USeg: " << cur_x << "/" << last_zoom << " -> " << x_at_zeropoint << "/0.0\n";
      CGAL::insert(this->arr, zu);
    }

    if (item->getLength() > MINIMUM_SEG_LENGTH) {
      cur_x -= item->getLength();
    }
    last_item = item;
  }
}


/*
void
TrajectoryFactory::zoom_trajectory()
{
  double current_zoom, current_speed;

  std::map<size_t, std::pair<double, double>> zoom_in;
  std::map<size_t, std::pair<double, double>> zoom_out;

  // Determine start zoom
  current_speed = this->result->items[0]->getSpeed();
  current_zoom = this->scomp.zoom_for_speed(current_speed);

  // Forward search: Where to scale down
  for (size_t i = 0 ; i < this->result->items.size() ; i++) {
    TrajectoryItem *item = this->result->items[i];
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      continue;
    }

    if (current_speed > item->getSpeed()) {
      // We must zoom in!
      double new_zoom = this->scomp.zoom_for_speed(item->getSpeed());
      zoom_in[i] = std::make_pair(current_zoom, new_zoom);
      current_zoom = new_zoom;
      current_speed = item->getSpeed();
    }
  }

  // Determine end zoom
  current_speed = this->result->items[this->result->items.size() - 1]->getSpeed();
  current_zoom = this->scomp.zoom_for_speed(current_speed);

  // Backward search: Where to scale up
  for (int i = this->result->items.size() - 1 ; i >= 0 ; i--) {
    TrajectoryItem *item = this->result->items[i];
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      continue;
    }

    if (current_speed > item->getSpeed()) {
      // We must zoom out! (Remember we're going backwards..)
      double new_zoom = this->scomp.zoom_for_speed(item->getSpeed());
      zoom_out[i] = std::make_pair(new_zoom, current_zoom);
      current_zoom = new_zoom;
      current_speed = item->getSpeed();
    }
  }

  // Determine start zoom
  current_speed = this->result->items[0]->getSpeed();
  current_zoom = this->scomp.zoom_for_speed(current_speed);

  // Actually build the zoom items
  std::vector<TrajectoryItem*> new_items;
  for (size_t i = 0 ; i < this->result->items.size() ; i++) {
    TrajectoryItem *item = this->result->items[i];
    if (item->getType() != TrajectoryItem::TYPE_STRAIGHT) {
      item->adjustZoomLevel(current_zoom);
      new_items.push_back(item);
      continue;
    }
    StraightTrajectoryItem *sitem = (StraightTrajectoryItem *)item;

    double zoom_in_finished = 0.0;
    double zoom_out_starts = sitem->getLength();

    /* First, only compute the distance along which we will zoom
    if (zoom_in.find(i) != zoom_in.end()) {
      // Zoom in!
      zoom_in_finished = std::min(ZOOM_MAX_PORTION * sitem->getLength(), this->scomp.seconds_to_meters_at_speed(sitem->getSpeed(), ZOOM_MAX_SECONDS));
    }

    if (zoom_out.find(i) != zoom_out.end()) {
      // Zoom out!
      zoom_out_starts = sitem->getLength() - std::min(ZOOM_MAX_PORTION * sitem->getLength(), this->scomp.seconds_to_meters_at_speed(sitem->getSpeed(), ZOOM_MAX_SECONDS));
    }

    // If both overlap, they meet in the middle!
    if (zoom_in_finished >= (zoom_out_starts - 0.1)) {
      zoom_in_finished = sitem->getLength() * 0.45;
      zoom_out_starts = sitem->getLength() * 0.55;
    }

    /* Now, actually replace the items
    bool free_original = false;
    // Zoom-in item
    if (zoom_in.find(i) != zoom_in.end()) {
      // Zoom in!
      double startZoom = zoom_in[i].first;
      double endZoom = zoom_in[i].second;

      Position start;
      Position end;
      if (sitem->getLength() < 0.01) {
        start = end = sitem->getStart();
      } else {
        start = sitem->getStart();
        CarPosition endCarpos = sitem->interpolatePosition(zoom_in_finished);
        end = endCarpos.pos;
      }

      StraightTrajectoryItem *zoomInItem = new StraightTrajectoryItem(start, end, startZoom, endZoom, sitem->getSpeed());
      new_items.push_back(zoomInItem);
      current_zoom = endZoom;
    }

    // No-Zoom item
    if ((zoom_in_finished == 0.0) && (zoom_out_starts == sitem->getLength())) {
      item->adjustZoomLevel(current_zoom);
      new_items.push_back(item); // Nothing changed
    } else {
      Position start;
      Position end;

      if (sitem->getLength() < 0.01) {
        start = end = sitem->getStart();
      } else {
        CarPosition startCarpos = sitem->interpolatePosition(zoom_in_finished);
        start = startCarpos.pos;
        CarPosition endCarpos = sitem->interpolatePosition(zoom_out_starts);
        end = endCarpos.pos;
      }

      StraightTrajectoryItem *noZoomItem = new StraightTrajectoryItem(start, end, current_zoom, current_zoom, sitem->getSpeed());
      new_items.push_back(noZoomItem);
      free_original = true;
    }

    // Zoom-out item
    if (zoom_out.find(i) != zoom_out.end()) {
      // Zoom out!
      double startZoom = zoom_out[i].first;
      double endZoom = zoom_out[i].second;

      Position start;
      Position end;

      if (sitem->getLength() < 0.01) {
        start = end = sitem->getStart();
      } else {
        CarPosition startCarpos = sitem->interpolatePosition(zoom_out_starts);
        start = startCarpos.pos;
        end = sitem->getEnd();
      }

      StraightTrajectoryItem *zoomOutItem = new StraightTrajectoryItem(start, end, startZoom, endZoom, sitem->getSpeed());
      new_items.push_back(zoomOutItem);
      current_zoom = endZoom;
    }

    if (free_original) {
      delete sitem;
    }
  }
  this->result->items = new_items;
}
*/
