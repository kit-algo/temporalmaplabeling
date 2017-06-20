/*
 * ILP.h
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#ifndef ILP_H_
#define ILP_H_
#include <vector>
#include <set>
#include <gurobi_c++.h>
#include "util/clock.h"
#include "Instance.h"
#include "adapter.h"

class ILPAdapter;

namespace ILP {
using namespace ILP;

inline long timevaldiff(struct timeval *starttime, struct timeval *finishtime)
{
  long msec;
  msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
  msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
  return msec;
}
	enum ModelType {AM1, AM2, AM3};
class ILP {
public:

	ILP();
	virtual ~ILP();

	bool execute(Instance * instance, ModelType modelType, vector<Intervals> & result, double &resultBound, double &gap, Clock & clock, int k, ILPAdapter *adapter, const char *dbg_filename);

private:
	vector<double>* createEvents(Instance * instance);
	void createVariables(GRBModel& model, int numberOfLabels,
			int numberOfSegments, vector<GRBVar*> &B, vector<GRBVar*>& X, vector<GRBVar*>& E, vector<GRBVar*>& V, vector<GRBLinExpr *> & constraints,
			vector<char *>& sense, vector<double *> &rhs,vector<const string *>& names, Instance *instance);


};

} // namespace ILP

#endif /* ILP_H_ */
