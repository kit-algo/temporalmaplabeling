#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <cassert>

class WeightedInterval{
private:
    int id_ =-1;
    double start_;
    double end_;
    double weight_;
public:
    double start() const {return start_;}
    double end()   const {return end_;}
    double weight() const {return weight_;}
    int    id() const {return id_;}

    double length() const {return end_-start_;}

    WeightedInterval(double start, double end, double weight) : id_(-1),
            start_(start), end_(end), weight_(weight){
        assert(start <= end);
    }

    WeightedInterval(int id, double start, double end, double weight) : id_(id),
            start_(start), end_(end), weight_(weight){
        assert(start <= end);
    }

    /**
     * Substracts the given intervals from this interval and returns the remaining intervals.
     * Assumes the given intervals are disjoint and ordered by their beginning from left to right
     */
    std::vector<WeightedInterval> substract(const std::vector<WeightedInterval> &intervals);
};


/**
 * Computes for a set of weighted intervals the maximum weight independent set on the according
 * interval graph. It returns the indices of the intervals contained in that set.
 * Needs O(n log n) time.
 */
std::vector<int> computeMWIS(std::vector<WeightedInterval> &intervals);

/**
 * Checks for a subset of intervals whether they are independent, i.e., whether they do not intersect
 * pairwise.
 * The subset is described by a set of indices on the given set of intervals.
 */
bool areIndependent(const std::vector<WeightedInterval> & intervals, const std::vector<int> & set);



#endif // UTILS_H

