/*
 * Instance.h
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_
#include <vector>
#include "Intervals.h"
#include "Conflict.h"
using namespace std;

namespace ILP {
using namespace ILP;

class Instance {
public:
	Instance();
	virtual ~Instance();
	vector<Intervals*> * intervalsOfLabels;
	vector<Intervals*> * intervalsOfConflicts;
	vector<Conflict*> *  conflicts;
	double originalMax;
};

} // namespace ILP

#endif /* INSTANCE_H_ */
