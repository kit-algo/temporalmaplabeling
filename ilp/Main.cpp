/*
 * Main.cpp
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */


#include <gurobi_c++.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <list>
#include <cstdlib>
#include <sys/time.h>
#include "Instance.h"
#include "Import.h"
#include "ILP.h"

using namespace std;

namespace ILP {

void findComponents(Instance * instance,vector<Instance*>& instances);

int
main(int   argc,
     char *argv[])
{
	string sLine = "";
	ifstream infile;

	//string file = "/home/benjamin/workspace/labelingTool/data/karlsruhe/presence/test.intervals";//argv[1];
	//string output = "/home/benjamin/workspace/labelingTool/data/karlsruhe/activity/test.intervals";//argv[2];


	//string file = "/home/benjamin/workspace/labelingTool/data/karlsruhe/presence/karlsruhe_city20.intervals";//argv[1];
	//string output = "/home/benjamin/workspace/labelingTool/data/karlsruhe/activity/karlsruhe_city20.intervals";//argv[2];
    ModelType type =AM3;
  string number = argv[1];
    string temp = argv[2];

    if(temp == "AM1"){
    	type = AM1;
    }else
    if(temp == "AM2"){
        	type = AM2;
    }else{
        	type = AM3;
    }
	string file = argv[3];
    string output = argv[4];
    string resultFile = argv[5];
	int k=-1;

	if(argc >6){

		k = atoi(argv[6]);
	}
	cout << "k: " << k << endl;
	//cout << file << endl;
    //cout << output << endl;
   //     return 0;

	Instance *instance = new Instance();
	Import import = Import();
	import.import(file,instance);

	infile.close();
	cout << "Read file completed!!" << endl;
	double phase1 = 0;
	double phase2 = 0;
	double phase3 = 0;
	vector<Instance*> instances = vector<Instance*>();

	timeval start, end;

	Clock clock1;
	clock1.start();
	if(k >= 0){
		 instances.push_back(instance);
	}else{
		findComponents(instance,instances);
	}

	phase1 = clock1.stop();

	vector<Intervals> intervals = vector<Intervals>();
	for(unsigned int i=0; i < instances.size(); i++){


		try{
			cout << "COMPONENT " << i << endl;
			if(instances[i]->intervalsOfLabels->size() == 1){
				 intervals.push_back(*instances[i]->intervalsOfLabels->at(0));
			}else{
				Clock clock2;

				ILP ilp = ILP();
				if(ilp.execute(instances[i],type,intervals,clock2,k)){
				  phase2 += clock2.getOverallTime();


				}else{
                   phase3 = -1;
                   break;
                }
			}


		}catch(GRBException & e){
			cerr << "ERROR:" << e.getMessage() << endl;
		}
		cout << "ILP has been executed" << endl;
	}
	cout << "EXPERIMENT FINISHED" << endl;




	cout << "COMPUTE VALUE"<<endl;
	double result =0;
	int intervalCount=0;
	for(vector<Intervals>::iterator it = intervals.begin(); it != intervals.end(); ++it){
			for(unsigned int i=0; i < (*it).size(); i+=2){
				result += (*it).at(i+1)-(*it).at(i);
				intervalCount++;
			}
	}
	cout << "OPT: " << result <<" " <<phase1+phase2+phase3 << " "<< intervalCount<< endl;
	if(output.compare("none")!=0){

	  import.exportIntervals(output,intervals);
	}


	if(resultFile.compare("none")!=0){
	fstream filestr;

	filestr.open (resultFile.c_str(), fstream::in | fstream::out | fstream::app);
	if(phase3 < 0){
               filestr << number<<" " << phase1 << " " << phase2 << " " << phase3 << " " << "TIMEOUT" << endl;
        }else{
               filestr << number<<" " << phase1 << " " << phase2 << " " << phase3 << " " << phase1+phase2+phase3 << endl;
	}


	filestr.close();
	}
//
//
//  try {
//    GRBEnv env = GRBEnv();
//
//    GRBModel model = GRBModel(env);
//
//    // Create variables
//
//    GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
//    GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
//    GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");
//
//    // Integrate new variables
//
//    model.update();
//
//    // Set objective: maximize x + y + 2 z
//
//    model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);
//
//    // Add constraint: x + 2 y + 3 z <= 4
//
//    model.addConstr(x + 2 * y + 3 * z <= 4, "c0");
//
//    // Add constraint: x + y >= 1
//
//    model.addConstr(x + y >= 1, "c1");
//
//    // Optimize model
//
//    model.optimize();
//
//    cout << x.get(GRB_StringAttr_VarName) << " "
//         << x.get(GRB_DoubleAttr_X) << endl;
//    cout << y.get(GRB_StringAttr_VarName) << " "
//         << y.get(GRB_DoubleAttr_X) << endl;
//    cout << z.get(GRB_StringAttr_VarName) << " "
//         << z.get(GRB_DoubleAttr_X) << endl;
//
//    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//
//  } catch(GRBException e) {
//    cout << "Error code = " << e.getErrorCode() << endl;
//    cout << e.getMessage() << endl;
//  } catch(...) {
//    cout << "Exception during optimization" << endl;
//  }

  return 0;
}

void findComponents(Instance * instance,vector<Instance*>& instances){
	vector<vector<int> > graph = vector<vector<int> >();
	int * component = new int[instance->intervalsOfLabels->size()];

	for(unsigned int i=0; i < instance->intervalsOfLabels->size(); i++){
		graph.push_back(vector<int>());
		component[i]=0;
	}

	for(vector<Conflict*>::iterator it = instance->conflicts->begin() ;
			it != instance->conflicts->end(); ++it){
		Conflict * c = *it;
		graph[c->getLabel1()].push_back(c->getLabel2());
		graph[c->getLabel2()].push_back(c->getLabel1());
	}

	list<int> queue = list<int>();

	int index =1;

	for(unsigned int i=0; i < instance->intervalsOfLabels->size(); i++){
		if(component[i]==0){

			queue.push_back(i);
			component[i] = index;
			while(!queue.empty()){
				int label = queue.front();
				queue.pop_front();
				for(vector<int>::iterator it =  graph[label].begin(); it != graph[label].end(); ++it){
					int opponent = *it;
					if(component[opponent]== 0){
					    component[opponent] = index;
					   	queue.push_back(opponent);

					}else if(component[opponent]!= index){
						throw "error!!!!";
					}
				}
			}

			index++;
		}

	}



	for(int i=0; i < index-1; i++){
		instances.push_back(new Instance());
	}

	int * newIds = new int[instance->intervalsOfLabels->size()];

	for(unsigned int i=0; i < instance->intervalsOfLabels->size(); i++){
		newIds[i] = instances[component[i]-1]->intervalsOfLabels->size();
		instances[component[i]-1]->intervalsOfLabels->push_back(instance->intervalsOfLabels->at(i));
	}



	for(unsigned int i=0; i < instance->conflicts->size(); i++){
		Conflict *c = instance->conflicts->at(i);
		if(component[c->label1] != component[c->label2]){
			throw "Wrong components";
		}
		int comp = component[c->label1];
		instances[comp-1]->conflicts->push_back(new Conflict(newIds[c->label1],newIds[c->label2]));

		instances[comp-1]->intervalsOfConflicts->push_back(instance->intervalsOfConflicts->at(i));
	}
//
//	cout << newIds[36] << " " << newIds[37] << endl;
//		cout << component[36] << " " << component[37] << endl;
//		Instance * test = instances[32];
//		for(int i=0; i  < test->intervalsOfLabels->size(); i++){
//			cout << test->intervalsOfLabels->at(i)->getId() << endl;
//		}
//		for(int i=0; i  < test->conflicts->size(); i++){
//			cout << test->intervalsOfConflicts->at(i)->getId() << endl;
//			cout << test->conflicts->at(i)->label1 << " " <<test->conflicts->at(i)->label2  << endl;
//		}
		//throw "check";

	delete [] component;
	delete [] newIds;

}

} // Namespace ILP
