/*
 * Instance.cpp
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */
#include<vector>
#include "Instance.h"

namespace ILP {

Instance::Instance() {
	  intervalsOfLabels = new vector<Intervals*>();
	  intervalsOfConflicts = new vector<Intervals*>();
	  conflicts = new vector<Conflict*>();

}

Instance::~Instance() {
	for(vector<Intervals*>::iterator it = intervalsOfLabels->begin(); it != intervalsOfLabels->end(); ++it){
		delete *it;
	}
	for(vector<Intervals*>::iterator it =intervalsOfConflicts->begin(); it != intervalsOfConflicts->end(); ++it){
		delete *it;
	}
	for(vector<Conflict*>::iterator it =conflicts->begin(); it != conflicts->end(); ++it){
		delete *it;
	}
	delete intervalsOfLabels;
	delete intervalsOfConflicts;
	delete conflicts;

}

} // namespace ILP
