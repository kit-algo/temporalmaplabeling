/*
 * Intervals.h
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#ifndef INTERVALS_H_
#define INTERVALS_H_

#include <sstream>
#include <iostream>
#include <vector>
using namespace std;


namespace ILP {
using namespace ILP;

/**
 * This class encapsulates all intervals of either a label or a conflict between two labels.
 */
class Intervals : public vector<double> {

private:
	int id;
    int numberOfIntervals;

public:
    Intervals(int id);
    virtual ~Intervals();

    int getNumberOfIntervals() const
    {
        return numberOfIntervals;
    }

    int getId() const
    {
        return id;
    }

    void add(double begin, double end)
    {
        numberOfIntervals++;
        this->push_back(begin);
        this->push_back(end);
    }
}

;

} // namespace ILP

#endif /* INTERVALS_H_ */
