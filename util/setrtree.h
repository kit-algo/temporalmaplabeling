#ifndef SETRTREE_H
#define SETRTREE_H

#include "boosthelper.h"

#include "map/map.h"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/concept/assert.hpp>
#include <boost/geometry/geometries/concepts/point_concept.hpp>

#include <iostream>
#include <fstream>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef boost::geometry::cs::cartesian coordinate_system_type;

typedef boost::geometry::model::point<double, 1, coordinate_system_type> Point1D;

inline bool operator==(const Point1D lhs, const Point1D rhs){ return lhs.get<0>() == rhs.get<0>(); }
inline bool operator!=(const Point1D lhs, const Point1D rhs){return !operator==(lhs,rhs);}
inline bool operator< (const Point1D lhs, const Point1D rhs){ return lhs.get<0>() < rhs.get<0>(); }
inline bool operator> (const Point1D lhs, const Point1D rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const Point1D lhs, const Point1D rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const Point1D lhs, const Point1D rhs){return !operator< (lhs,rhs);}

inline bool operator==(const std::pair<Point1D, Point1D> &lhs, const std::pair<Point1D, Point1D> &rhs){ return lhs.first.get<0>() == rhs.first.get<0>() && lhs.second.get<0>() == rhs.second.get<0>(); }
inline bool operator!=(const std::pair<Point1D, Point1D> &lhs, const std::pair<Point1D, Point1D> &rhs){return !operator==(lhs,rhs);}
inline bool operator< (const std::pair<Point1D, Point1D> &lhs, const std::pair<Point1D, Point1D> &rhs){ return (lhs.first.get<0>() < rhs.first.get<0>()) || (lhs.first.get<0>() == rhs.first.get<0>() && lhs.second.get<0>() < rhs.second.get<0>()); }
inline bool operator> (const std::pair<Point1D, Point1D> &lhs, const std::pair<Point1D, Point1D> &rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const std::pair<Point1D, Point1D> &lhs, const std::pair<Point1D, Point1D> &rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const std::pair<Point1D, Point1D> &lhs, const std::pair<Point1D, Point1D> &rhs){return !operator< (lhs,rhs);}


typedef boost::geometry::model::segment<Point1D> Interval;
typedef boost::geometry::index::rtree<Interval, boost::geometry::index::linear<4>> Tree;

std::ostream &operator<<(std::ostream &os, const Interval &i);

/* std::less fails to use the operators defined below. Please don't ask me why *sob*. */
struct IntervalComp {
  bool operator()(const Interval& lhs, const Interval& rhs) const {
    return (lhs.first.get<0>() < rhs.first.get<0>()) || (lhs.first.get<0>() == rhs.first.get<0>() && lhs.second.get<0>() < rhs.second.get<0>());
  }
};

inline bool compareIntervals(const Interval& lhs, const Interval& rhs) {
  return (lhs.first.get<0>() < rhs.first.get<0>()) || (lhs.first.get<0>() == rhs.first.get<0>() && lhs.second.get<0>() < rhs.second.get<0>());
}

/* For pairs / intervals. Because GCC fails to infer this. Fuck templates, seriously. */
inline bool operator==(const Interval &lhs, const Interval &rhs){ return lhs.first.get<0>() == rhs.first.get<0>() && lhs.second.get<0>() == rhs.second.get<0>(); }
inline bool operator!=(const Interval &lhs, const Interval &rhs){return !operator==(lhs,rhs);}
inline bool operator< (const Interval &lhs, const Interval &rhs){ return (lhs.first.get<0>() < rhs.first.get<0>()) || (lhs.first.get<0>() == rhs.first.get<0>() && lhs.second.get<0>() < rhs.second.get<0>()); }
inline bool operator> (const Interval &lhs, const Interval &rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const Interval &lhs, const Interval &rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const Interval &lhs, const Interval &rhs){return !operator< (lhs,rhs);}



inline Interval make_interval(double p1, double p2) {
  return Interval(Point1D(p1), Point1D(p2));
}

inline bool intervals_overlap(Interval const & i1, Interval const & i2) {
  return !(boost::geometry::detail::disjoint::disjoint_segment_1d<Interval, Interval>::apply(i1, i2));
}


/*
 * This is a list of things that should be serializable below...
 */
inline std::ostream& operator<< (std::ostream &out, const std::set<POI *> &val) {
  out << "{";
  bool first = true;
  for (auto poi : val) {
    if (!first) {
      out << ", ";
    } else {
      first = false;
    }
    out << poi->getId();
  }
  out << "}";

  return out;
}

inline std::ostream& operator<< (std::ostream &out, const std::set<std::set<POI *>> &val) {
  out << "{";
  bool first = true;
  for (std::set<POI *> pair : val) {
    if (!first) {
      out << ", ";
    } else {
      first = false;
    }
    out << "(";
    out << (*(pair.begin()))->getId();
    out << ",";
    out << (*(++pair.begin()))->getId();
    out << ")";
  }
  out << "}";

  return out;
}


template<class Value>
class SetRTree {
public:
  void clear() {
    this->tree.clear();
    this->values.clear();
  }

  void insert(Interval interval, Value val) {
    if (this->values.find(interval) == this->values.end()) {
      this->tree.insert(interval);
      this->values[interval] = std::set<Value>({ val });
    } else {
      this->values[interval].insert(val);
    }
  }

  size_t size() {
    return this->tree.size();
  }

  size_t full_size() {
    size_t n = 0;
    for (auto entry : this->values) {
      n += entry.second.size();
    }
    return n;
  }

  template<typename Predicates>
  std::set<Value> query(Predicates const & predicates) {
    std::set<Interval, IntervalComp> resIntervals;
    this->tree.query(predicates, std::inserter<std::set<Interval, IntervalComp>>(resIntervals, resIntervals.begin()));
    std::set<Value> res;
    for (auto interval : resIntervals) {
      res.insert(this->values[interval].cbegin(), this->values[interval].cend());
    }

    return std::move(res);
  }

  template<typename Predicates>
  std::vector<Interval> queryIntervals(Predicates const & predicates) {
    std::vector<Interval> resIntervals;
    this->tree.query(predicates, std::back_inserter<std::vector<Interval>>(resIntervals));

    return std::move(resIntervals);
  }

  template<typename Predicates>
  std::vector<Interval> queryIntervalsSorted(Predicates const & predicates) {
    std::vector<Interval> resIntervals;
    this->tree.query(predicates, std::back_inserter<std::vector<Interval>>(resIntervals));
    std::sort(resIntervals.begin(), resIntervals.end(), compareIntervals);

    return std::move(resIntervals);
  }

  std::set<Value> findAtPoint(double point) const {
    Interval lookup = make_interval(point, point);
    return std::move(this->query(bgi::intersects(lookup)));
  }

  void write(const char *filename) {
    std::ofstream outfile;
    outfile.open (filename);

    for (auto interval: this->queryIntervalsSorted(bgi::satisfies([](Interval const&){ return true; }))) {
      double start = bg::get<0>(interval.first);
      double end = bg::get<0>(interval.second);
      outfile << start << "," << end << ",\"";
      outfile << this->values[interval];
/*      for (auto val : this->values[interval]) {
        outfile << val << ",";
      }*/
      outfile << "\"\n";
    }
    outfile.close();
  }

  void dbg_output() {
    std::cout << " ===========DBG============\n";
    for (auto interval: this->queryIntervalsSorted(bgi::satisfies([](Interval const&){ return true; }))) {
      double start = bg::get<0>(interval.first);
      double end = bg::get<0>(interval.second);
      std::cout << "=> " << start << "->" << end << ":\n";
      for (auto val : this->values[interval]) {
        std::cout << "   ~> " << val << "\n";
      }
    }
    std::cout << " ===========END============\n";
  }

  template<typename Predicates>
  std::vector<std::pair<Interval, Value>> queryFull(Predicates const & predicates) {
    std::vector<Interval> resIntervals;
    this->tree.query(predicates, std::back_inserter<std::vector<Interval>>(resIntervals));
    std::vector<std::pair<Interval, Value>> res;
    for (auto interval : resIntervals) {
      for (auto poi : this->values[interval]) {
        res.push_back(std::make_pair(interval, poi));
      }
    }

    return std::move(res);
  }

  template<typename Predicates>
  std::vector<std::pair<Interval, Value>> queryFullSorted(Predicates const & predicates) {
    std::vector<Interval> resIntervals;
    this->tree.query(predicates, std::back_inserter<std::vector<Interval>>(resIntervals));
    std::sort(resIntervals.begin(), resIntervals.end(), compareIntervals);

    std::vector<std::pair<Interval, Value>> res;
    for (auto interval : resIntervals) {
      for (auto val : this->values[interval]) {
        res.push_back(std::make_pair(interval, val));
      }
    }

    return std::move(res);
  }

private:
  Tree tree;
  std::map<Interval, std::set<Value>, IntervalComp> values;
};

#endif
