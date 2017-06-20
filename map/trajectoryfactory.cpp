#include "trajectoryfactory.h"

#include "util/coordinateutil.h"

#include <math.h>
#include <algorithm>
#include <armadillo>
#include <assert.h>

TrajectoryFactory::TrajectoryFactory(std::vector<Position> *waypoints, std::vector<double> *speeds, double w, double h, Map *map):
    result(NULL), w(w), h(h), map(map), waypoints(waypoints), speeds(speeds)
{
}

void TrajectoryFactory::zoom_trajectory() {
  ZoomComputer zc(this->result, this->w, this->h, this->map);
  zc.run();
}

Trajectory* TrajectoryFactory::getTrajectory()
{
  if (this->result == NULL) {
    this->result = new Trajectory();
    this->build_unzoomed_trajectory();
    std::cout << "============================================\n";
    std::cout << "Built unzoomed trajectory: \n";
    this->result->dbg_print();
    std::cout << "============================================\n";
    this->zoom_trajectory();
  }

  return this->result;
}

// This is a safety margin
#define ZOOM_MAX_PORTION 0.95
// Maximum time that zooming should take
#define ZOOM_MAX_SECONDS 5.0


void
TrajectoryFactory::build_unzoomed_trajectory()
{
    std::vector<Position>::iterator wp_it = this->waypoints->begin();
    std::vector<double>::iterator speed_it = this->speeds->begin();

    double cur_speed, next_speed;
    Position cur;
    Position last = cur = *wp_it;
    cur_speed = *speed_it;
    Position last_touched = last;
    wp_it++;
    speed_it++;
    Position next = *wp_it;
    next_speed = *speed_it;
    wp_it++;
    speed_it++;
    double last_covered = 0.0;
    double last_length;
    double next_length = Map::compute_distance_latlon(cur, next);
    double available = 0.0;

    for ( ; wp_it != this->waypoints->end(); wp_it++, speed_it++) {
        last = cur;
        cur = next;
        next = *wp_it;
        cur_speed = next_speed;

        while (next == cur) {
            wp_it++;
            speed_it++;
            next = *wp_it;
            next_speed = *speed_it;
        }

        last_length = next_length;
        next_length = Map::compute_distance_latlon(cur, next);

        std::cout << "Making segment: length is " << next_length << "\n";

        last_covered = available;
        double last_available = last_length - last_covered;
        available = std::min(last_available, (next_length / 2.0));

        double circle_speed = std::min(next_speed, cur_speed);
        CircleTrajectoryItem *circle = this->compute_circle(cur, last, next, available, circle_speed);
        if (circle == nullptr) {
          continue;
        }


        // Make a straight item to this circle
        CarPosition circleStart = circle->interpolatePosition(0.0);
        std::cout << "Circle starts at: " << circleStart.pos.first << "," << circleStart.pos.second << "\n";

        StraightTrajectoryItem *straight = new StraightTrajectoryItem(last_touched, circleStart.pos, 1.0, 1.0, cur_speed);
        std::cout << "Straight item: " << last_touched.first << "," << last_touched.second << " -> " << circleStart.pos.first << "," << circleStart.pos.second << "\n";

        this->result->addItem(straight);
        this->result->addItem(circle);
        last_touched = circle->interpolatePosition(circle->getLength()).pos;
        std::cout << "Circle ends at " << last_touched.first << "," << last_touched.second << "\n";


    }

    StraightTrajectoryItem *straight = new StraightTrajectoryItem(last_touched, next, 1.0, 1.0, cur_speed);
    this->result->addItem(straight);
}

CircleTrajectoryItem *TrajectoryFactory::compute_circle(Position cur, Position last, Position next, double len, double speed)
{
    arma::vec vec_base({cur.first, cur.second});

    arma::vec vec_last ({last.first - cur.first, last.second - cur.second});
    vec_last = arma::normalise(vec_last);
    vec_last = vec_last * len;
    arma::vec vec_next ({next.first - cur.first, next.second - cur.second});
    vec_next = arma::normalise(vec_next);
    vec_next = vec_next * len;

    double angle_between = acos(arma::norm_dot(vec_last, vec_next));

    if ((angle_between > (M_PI - 0.03) && (angle_between < (M_PI + 0.03))))
            return nullptr;

    double alpha = (M_PI / 2.0) - (angle_between / 2.0);
    double center_dist = len / sin(alpha);
    double radius = len / tan(alpha);

    arma::vec center_direction_vec = vec_last + vec_next;
    center_direction_vec = arma::normalise(center_direction_vec);
    center_direction_vec *= center_dist;

    arma::vec center_vec = vec_base + center_direction_vec;
    Position center (center_vec(0), center_vec(1));

    //std::cout << "Last: " << last;
    //std::cout << "Cur: " << cur;
    //std::cout << "Next: " << next;

    double start_angle = CoordinateUtil::getAngle(last, cur);
    double end_angle = CoordinateUtil::getAngle(cur, next);

    //std::cout << "Start angle: " << start_angle;
    //std::cout << "End angle: " << end_angle;

    // start_angle and end_angle are tangential to the circle!
    // We have to convert this.
    // Step 1: Find out if we're bending left or right
    bool bendLeft = (CoordinateUtil::angleDiff(start_angle, end_angle) > 0);

    if (bendLeft) {
        // The circle is right of the bend, i.e. the circle's angle is
        // 90° clockwise from the tangential
        start_angle -= (M_PI/2.0);
        if (start_angle < -1*M_PI) {
            start_angle += 2*M_PI;
        }
        end_angle -= (M_PI/2.0);
        if (end_angle < -1*M_PI) {
            end_angle += 2*M_PI;
        }
    } else {
        start_angle += (M_PI/2.0);
        if (start_angle > M_PI) {
            start_angle -= 2*M_PI;
        }
        end_angle += (M_PI/2.0);
        if (end_angle > M_PI) {
            end_angle -= 2*M_PI;
        }
    }

    CircleTrajectoryItem *circle = new CircleTrajectoryItem(center, radius, start_angle, end_angle, 1.0, speed);
    return circle;
}
