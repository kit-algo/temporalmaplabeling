/*
 * ILP.cpp
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */
#include <set>
#include <gurobi_c++.h>
#include <sys/time.h>
#include "Intervals.h"
#include "ILP.h"
#include "config.h"


#define NC 4
#define CONTAINED 0
#define LEFT      1
#define RIGHT     2

using namespace std;

namespace ILP {

ILP::ILP() {
	// TODO Auto-generated constructor stub

}

ILP::~ILP() {
	// TODO Auto-generated destructor stub
}

/**
 * This function assumes that the interval [x1,x2] is a atomic segment.
 * It checks whether [x1,x2] is contained in, lies left or lies right to
 * [y1,y2].
 */
int compare(double x1, double x2, double y1, double y2){

	if(y1 <= x1 && x2 <= y2){ // interval [x1,x2] is subset of [y1,y2]
	   return CONTAINED;
	}
	if(x2 <= y1){
		return LEFT;
	}
	return RIGHT;
}


string convert(int number)
{
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

string convert(double number)
{
   std::stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

bool ILP::execute(Instance* instance, ModelType modelType, vector<Intervals> & result,
							double &resultBound, double &gap, Clock & clock, int k, ILPAdapter *adapter, const char *dbg_filename) {

	clock.start();

	vector<double>* events = createEvents(instance);

	GRBEnv env = GRBEnv();
  env.set(GRB_DoubleParam_TimeLimit, TIME_LIMIT);
	env.set(GRB_IntParam_PrePasses, PRESOLVE_ITERATIONS);
  env.set(GRB_IntParam_Threads,num_threads);
	env.set(GRB_IntParam_LogToConsole, 1);

	env.set(GRB_IntParam_Presolve, 1);

	/* This is necessary because of a bug in Gurobi up to version 6.5. Supposedly it
	 * will be fixed in later versions, see:
	 * https://groups.google.com/forum/#!topic/gurobi/JJl0ptJZwzE
	 */
	env.set(GRB_IntParam_Aggregate, 0);

	//env.set(GRB_StringParam_LogFile, "/tmp/gurobi.log");

	GRBModel model = GRBModel(env);

	// Variables indicating where a visibility begins
	vector<GRBVar *> B = vector<GRBVar*>();
	// Variables indicating where a visibility is switched on
	vector<GRBVar *> X = vector<GRBVar*>();
	// Varibales indicating where a visibility ends
	vector<GRBVar *> E = vector<GRBVar*>();
	// Variables indicating whether a presence is selected at all
	vector<GRBVar *> V = vector<GRBVar*>();

	vector<GRBLinExpr *> constraints = vector<GRBLinExpr*>();
	vector<char *> sense = vector<char*>();
	vector<double *> rhs = vector<double*>();
	vector<const string *> names = vector<const string*>();





	vector<vector<int> > beginOfConflicts = vector<vector<int> >(events->size()-1);
	vector<vector<int> > endOfConflicts = vector<vector<int> >(events->size()-1);





	createVariables(model,instance->intervalsOfLabels->size(),events->size()-1, B, X, E, V, constraints,sense,rhs,names, instance);



	for(unsigned int event=0; k>=0&&event < events->size()-1; ++event){
		GRBLinExpr expr;
		for(vector<GRBVar *>::iterator label = X.begin(); label != X.end(); ++label ){
			expr += (*label)[event];

		}
		model.addConstr(expr,GRB_LESS_EQUAL,k,"ck_"+convert((int)event));
	}

	cerr << "Passed Checkpoint 1.1\n";

	/**
	 * conditions only based on conflict intervals
	 */
	int conflictIndex =0;
	for(vector<Intervals*>::iterator it = instance->intervalsOfConflicts->begin() ;
			it != instance->intervalsOfConflicts->end(); ++it,++conflictIndex){
		Intervals * intervals = (*it);
		int conflictEvent = 0;
		double beginConflict = intervals->at(conflictEvent*2);
		double endConflict  = intervals->at(conflictEvent*2+1);


		GRBLinExpr sumB, sumE;
		int state =0;



		for(unsigned int event=0; event < events->size()-1; ++event){

			double beginSegment = events->at(event);
			double endSegment   = events->at(event+1);


			if(beginConflict == beginSegment){
				beginOfConflicts[event].push_back(conflictIndex);
			}

			if(endConflict == endSegment){
				endOfConflicts[event].push_back(conflictIndex);
			}


			if(compare(beginSegment,endSegment,beginConflict, endConflict) == CONTAINED){

				Conflict *conflict = instance->conflicts->at(conflictIndex);

				int label1 = conflict->getLabel1();
				int label2 = conflict->getLabel2();


				model.addConstr((X.at(label1)[event]+X.at(label2)[event])<=1,
								 "c8_"+convert(conflictIndex)+"_"+convert(label1)+"_"+convert(label2));


			}


			if(endConflict == endSegment){ // Although both are of double this comparision is okay,
				// because events and intervals contain identical objects.
				conflictEvent+=1;
				if(conflictEvent < (int)intervals->getNumberOfIntervals()){
					beginConflict = intervals->at(conflictEvent*2);
					endConflict = intervals->at(conflictEvent*2+1);
				}else{
					beginConflict = 2.0;
					endConflict = 2.0;
				}

			}
		}

	 }

	/**
	 * conditions only based on presence intervals.
	 */
	int label=0;

	for(vector<Intervals*>::iterator it = instance->intervalsOfLabels->begin() ;
			it != instance->intervalsOfLabels->end(); ++it,++label){

		Intervals * intervals = (*it);
		int presence = 0;
		double beginPresence = intervals->at(presence*2);
		double endPresence = intervals->at(presence*2+1);

		//cout << "#########LABEL " << adapter->poiForId(label)->getId() << endl;
		//cout << "#########Interal ID " << label << endl;

		//cout << "Presence is now " << beginPresence << " / " << endPresence << "\n";

		GRBLinExpr sumB, sumE, minLengthConstr;
		int state =0;

		for(unsigned int event=0; event < events->size()-1; ++event) {
			cout.flush();
			double beginSegment = events->at(event);
			double endSegment   = events->at(event+1);

			string name = convert(label)+"_"+convert((int)event);
			int index = event*NC;

			// condition (D)


			if(event==0){

			//	constraints[label][index+3] = B[label][event]-X[label][event];


				model.addConstr(B[label][event]==X[label][event],"c4_"+name);
			}else{

				//constraints[label][index+3] = B[label][event]+X[label][event-1]-E[label][event-1]-X[label][event];
				model.addConstr(B[label][event]+X[label][event-1]==E[label][event-1]+X[label][event],
								 "c5_"+name);
			}



			rhs[label][index+3] = 0.0;

			sense[label][index+3] = GRB_EQUAL;
		//	const string temp = name + "_D";
			//names[label][index+3] = name;
			//names[label][index+3] = "D";


			switch(compare(beginSegment,endSegment,beginPresence, endPresence)){
			case CONTAINED:
				if(state != 1){
				//	cout << "CONTAINED"<<endl;
					state =1;
				}
				if(beginSegment == beginPresence){
					sumB = GRBLinExpr();
					sumE = GRBLinExpr();
					sumE += V[label][presence];
					minLengthConstr = GRBLinExpr();
					minLengthConstr += V[label][presence] * (MINIMUM_SELECTION_LENGTH + 1.0);
					//minLengthConstr += (MINIMUM_SELECTION_LENGTH + 1.0);
				}

				minLengthConstr += (endSegment - beginSegment);
				assert((endSegment - beginSegment) >= 0.0);

				sumB += B[label][event];
				sumE += E[label][event];

				if(endSegment == endPresence){
					model.addConstr(sumB <= 1, "c6_"+name);
					model.addConstr(sumE <= 1, "c7_"+name);
				}
				if( endSegment != endPresence){
					GRBLinExpr witnessEnd   = GRBLinExpr();
					witnessEnd   += 0;


					if(modelType == AM2 || modelType == AM3){
						for(unsigned int i=0; i< beginOfConflicts[event+1].size(); ++i){
							Conflict *conflict = instance->conflicts->at(beginOfConflicts[event+1][i]);
							if(conflict->involves(label)){
								witnessEnd += X[conflict->getOpponent(label)][event+1];
							}
						}
					}
					model.addConstr(E[label][event]<=witnessEnd  , "c_10"+name);

				}


				if(beginSegment != beginPresence){
					GRBLinExpr witnessBegin = GRBLinExpr();

					witnessBegin += 0;

                                        if(modelType == AM3){
					 for(unsigned int i=0; i< endOfConflicts[event-1].size(); ++i){
						Conflict *conflict = instance->conflicts->at(endOfConflicts[event-1][i]);
						if(conflict->involves(label)){
							witnessBegin += X[conflict->getOpponent(label)][event-1];
					 	}
					 }
					}

					model.addConstr(B[label][event]<=witnessBegin  , "c_9"+name);
				}


				break;
			case LEFT: // there is no presence interval that contains the atomic segment.
				if(state != 2){

								state =2;
							}



				model.addConstr((B.at(label)[event])==0, "c1_"+convert(label)+"_"+convert((int)event));
				model.addConstr((X.at(label)[event])==0, "c2_"+convert(label)+"_"+convert((int)event));
				model.addConstr((E.at(label)[event])==0, "c3_"+convert(label)+"_"+convert((int)event));
				break;
			case RIGHT:
				throw "The wrong atomic segment is considered.";
				break;
			}

			if(endPresence == endSegment){ // Although both are of double this comparision is okay,
				                           // because events and intervals contain identical objects.
				assert(instance->originalMax >= 1.0);
				//std::cout << "Begin: " << beginPresence << " -- Ende: " << endPresence << "\n";
				//std::cout << minLengthConstr << " >= " << (MINIMUM_SELECTION_LENGTH / instance->originalMax) << "\n";
				model.addConstr(minLengthConstr >= (MINIMUM_SELECTION_LENGTH / instance->originalMax));
				minLengthConstr = GRBLinExpr();

				presence+=1;

				if(presence < (int)intervals->getNumberOfIntervals()){
					beginPresence = intervals->at(presence*2);
					endPresence = intervals->at(presence*2+1);
				}else{
					beginPresence = 2.0;
					endPresence = 2.0;
				}

			}
		}
	}
	cerr << "Passed Checkpoint 1.2\n";
//	cout << "ADD CONSTRAINTS "<< constraints.size() << endl;
//	cout << (events->size()-1)*NC << endl;
//
//	for(unsigned int label =0; label < constraints.size(); label++){
//		cout << "add" << label << endl;
//		model.addConstrs(constraints[label],sense[label],rhs[label],names[label],(events->size()-1)*NC);
//
//	}
	/**
	 * objective function
	 */
	label=0;
	GRBLinExpr linExpr = GRBLinExpr();
	for(unsigned int i=0; i < instance->intervalsOfLabels->size(); ++i){
		//GRBLinExpr minLengthConstr;
		for(unsigned int event=0; event < events->size()-1; ++event){
			double beginSegment = events->at(event);
			double endSegment   = events->at(event+1);
			linExpr += (endSegment-beginSegment)* X[i][event];
			//minLengthConstr += (endSegment-beginSegment)* X[i][event];
		}

		//minLengthConstr += V[i][0] * MINIMUM_SELECTION_LENGTH;
		//model.addConstr(minLengthConstr >= (MINIMUM_SELECTION_LENGTH / instance->originalMax), "minlength_" + i);
	}
	model.setObjective(linExpr,GRB_MAXIMIZE);


	if (dbg_filename != nullptr) {
		model.update();
		model.write(dbg_filename);
	}

	std::cout << "Starting to optimize...\n";
	model.optimize();
	std::cout << "Optimization done...\n";

	clock.stop();

//	for(unsigned int i=0; i < X.size(); i++){
//		cout << "Label " << i << ": ";
//		for(unsigned int j=0; j < events->size()-1; j++){
//			cout << X[i][j].get(GRB_DoubleAttr_X) << " ";
//		}
//		cout << endl;
//	}

	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL){
	//result->reserve(instance->intervalsOfLabels->size());
	for(unsigned int i =0; i < instance->intervalsOfLabels->size(); ++i){
		double begin =0.0;
		int state =0;
		Intervals intervals = Intervals(instance->intervalsOfLabels->at(i)->getId());
		for(unsigned int event=0; event < events->size()-1; ++event){
			if(state != 1 && X[i][event].get(GRB_DoubleAttr_X)==1){
				begin = events->at(event);
				state = 1;
			}
			if(state != 0 && X[i][event].get(GRB_DoubleAttr_X)==0){
				intervals.add(begin,events->at(event));
				state =0;
			}
		}
		if(state == 1){
			intervals.add(begin,1.0);
		}
		result.push_back(intervals);
	}
	}
	resultBound = model.get(GRB_DoubleAttr_ObjBound);
	gap = model.get(GRB_DoubleAttr_MIPGap);

  if (model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT){
		return false;
	}
	return true;

}



void add(vector<Intervals*> * container,set<double>& events ){
	for(vector<Intervals*>::iterator it = container->begin(); it != container->end(); ++it){
		for(vector<double>::iterator eventIt = (*it)->begin(); eventIt != (*it)->end(); ++eventIt){
			events.insert(*eventIt);
		}
	}
}



vector<double>* ILP::createEvents(Instance* instance) {
	set<double> eventSet = set<double>();
	add(instance->intervalsOfConflicts,eventSet);
	add(instance->intervalsOfLabels,eventSet);
	eventSet.insert(0.0);
	eventSet.insert(1.0);

	vector<double> * events = new vector<double>();
	events->reserve(eventSet.size());
	for(set<double>::iterator it = eventSet.begin(); it != eventSet.end(); ++it){
		events->push_back(*it);
	}
	return events;
}

void ILP::createVariables(GRBModel& model, int numberOfLabels,
		int numberOfSegments, vector<GRBVar*> &B, vector<GRBVar*>& X, vector<GRBVar*>& E, vector<GRBVar*>& V,
		vector<GRBLinExpr *> & constraints,vector<char *>& sense,vector<double *>& rhs,vector<const string *>& names, Instance * instance) {
	//B = new GRBVar[numberOfLabels][numberOfSegments];
	//X = new GRBVar[numberOfLabels][numberOfSegments];
	//E = new GRBVar[numberOfLabels][numberOfSegments];
	B.resize(numberOfLabels);
	X.resize(numberOfLabels);
	E.resize(numberOfLabels);
	V.resize(numberOfLabels);
	constraints.resize(numberOfLabels*NC);
	rhs.resize(numberOfLabels*NC);
	names.resize(numberOfLabels*NC);
	sense.resize(numberOfLabels*NC);


	for(int i=0; i < numberOfLabels; i++){
		B[i] = model.addVars(numberOfSegments,GRB_BINARY);
		X[i] = model.addVars(numberOfSegments,GRB_BINARY);
		E[i] = model.addVars(numberOfSegments,GRB_BINARY);
		V[i] = model.addVars(instance->intervalsOfLabels->at(i)->size(), GRB_BINARY);
		int number = numberOfSegments*NC;
		constraints[i] = new GRBLinExpr[number];
		rhs[i] = new double[number];
		names[i] = new string[number];
		sense[i] = new char[number];
	}

	model.update();

}

} // namespace ILP
