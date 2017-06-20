/*
 * Clock.h
 *
 *  Created on: Sep 4, 2013
 *      Author: benjamin
 */


#ifndef CLOCK_H_
#define CLOCK_H_
#include <list>
#include <sys/time.h>
#include <iostream>
#include <iomanip>
//#include <time.h>
using namespace std;


/**
 * Helper class that wraps a mechanism for measuring time.
 */
class Clock {
private:
	double overallTime;
    timeval startTime;
    list<double> times;
    bool startNecessary;
public:
	Clock() {};
	virtual ~Clock() {};

	void start(){
		  gettimeofday(&startTime, 0);
		  startNecessary = false;
	}

	/**
	 * Measures the time elapsed since calling method start and stores this time.
	 */
	double stop(){
		if(startNecessary){throw "start must be called before";}
		startNecessary = true;
		timeval end;
		gettimeofday(&end, 0);
		double time= ((double)end.tv_sec+(double)end.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
		if(time < 0.0001){time =0;}
		times.push_back(time);
		overallTime += time;
		return time;
	}

	double get(){
		if(startNecessary){throw "start must be called before";}
		timeval end;
		gettimeofday(&end, 0);
		double time= ((double)end.tv_sec+(double)end.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
		return time;
	}

	double getOverallTime() const{
		return overallTime;
	}

	void write(ostream & stream) {
		for(list<double>::iterator it= times.begin(); it != times.end(); ++it){

			stream << " " << *it;
		}
	}


};

#endif /* CLOCK_H_ */
