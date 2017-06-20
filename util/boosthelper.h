#ifndef BOOST_HELPER_H
#define BOOST_HELPER_H

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/geometries/segment.hpp>

#ifndef OLD_BOOST
#include <boost/geometry/algorithms/dispatch/disjoint.hpp>
#else
#include <boost/geometry/algorithms/disjoint.hpp>
#endif


namespace boost { namespace geometry
{
namespace detail { namespace disjoint
{

template <typename Segment1, typename Segment2>
struct disjoint_segment_1d
{
    static inline bool apply(Segment1 const& segment1, Segment2 const& segment2)
    {
        //typedef typename point_type<Segment1>::type point_type;

        auto start1 = segment1.first;
        auto end1 = segment1.second;

        auto start2 = segment2.first;
        auto end2 = segment2.second;

        return ((boost::geometry::get<0>(end1) < boost::geometry::get<0>(start2)) || (boost::geometry::get<0>(start1) > boost::geometry::get<0>(end2)));
    }
};


}} // namespace detail::disjoint

namespace dispatch
{

template <typename Segment1, typename Segment2>
struct disjoint<Segment1, Segment2, 1, segment_tag, segment_tag, false>
    : detail::disjoint::disjoint_segment_1d<Segment1, Segment2>
{};

} // namespace dispatch

}} // Namespace boost::geometry

#endif
