#include "setrtree.h"

// Make interval serializable to outstreams
std::ostream &operator<<(std::ostream &os, const Interval &i) {
    return os << "[" << boost::geometry::get<0>(i.first) << " / " << boost::geometry::get<0>(i.second) << "]";
}
