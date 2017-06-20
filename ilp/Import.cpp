/*
 * Import.cpp
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include "Import.h"
#include "Instance.h"
#include "Intervals.h"
using namespace std;

namespace ILP {

Import::Import() {
	// TODO Auto-generated constructor stub

}

Import::~Import() {
	// TODO Auto-generated destructor stub
}


Intervals * Import::readIntervals(string line) {
	istringstream iss(line);
	string word;
	int i=0;
	int size =0;
	int id;
	double begin;
	double end;
	Intervals * intervals =0;
	int index =0;
	while( iss >> word )
	{
		stringstream ss(word);
		if(i==0){
			if((ss >> id).fail() ){
				throw "Could not read id.";
			}
		    intervals = new Intervals(id);
		}
		if(i==1){
			if( (ss >> size).fail() ){
				throw "Could not read the number of intervals.";
			}
		}
		if(i>1 && i % 2 == 0){
			if((ss >> begin).fail() ){
					throw "Could not read begin.";
			}
		}
		if(i>1 && i % 2 == 1){
			if((ss >> end).fail() ){
						throw "Could not read end.";
			}

		    intervals->add(begin,end);
		//    cout << "add " << begin <<  " " << end << " " << intervals->getNumberOfIntervals()<< endl;

		    index++;
		}
		i++;
	}
	return intervals;
}

Conflict * Import::readConflict(string line) {
	istringstream iss(line);
	string word1;
	string word2;
	int id1;
	int id2;
	if(!(iss >> word1) || !(iss >> word2)){
		return 0;
	}

	stringstream ss1(word1);
	stringstream ss2(word2);
	if((ss1 >> id1).fail() || (ss2 >> id2).fail()){
							throw "Could not read ids.";
	}
	return new Conflict(id1, id2);

}

void Import::import(string file, Instance * instance) {

		ifstream infile;

		infile.open(file.c_str());
		string state = "";
		bool readSize = false;
		int size =0;
		while (!infile.eof())
		{
			string line;
			getline(infile, line);
//			cout << line << endl;

			if(line.size() > 0 &&line.at(0) == '#'){
				state = line;
				readSize = true;
			}else if(readSize){
				stringstream ss(line);
				if( (ss >> size).fail() ){
					throw "Could not read the size of the next block.";
				}
				readSize = false;
			}else if(state == "#INTERVALS FOR CONFLICTS"){
				Intervals *intervals = readIntervals(line);
				if(intervals!= 0){
					instance->intervalsOfConflicts->push_back(intervals);
				}
			}else if(state == "#INTERVALS FOR LABELS"){
				Intervals * intervals = readIntervals(line);
				if(intervals != 0){
					instance -> intervalsOfLabels->push_back(intervals);
				}
			}else if(state == "#CONFLICTS"){
				Conflict * conflict = readConflict(line);
				if(conflict != 0){
					instance->conflicts->push_back(conflict);
				}
			}else if(state == "#END"){
				return;
			}
		}

}

void Import::exportIntervals(string file, vector<Intervals>& intervals) {

	ofstream os;

	os.open(file.c_str());

	os << "#INTERVALS FOR LABELS" << endl;
	os << intervals.size() << endl;
	for(vector<Intervals>::iterator it = intervals.begin(); it != intervals.end(); ++it){
		os << (*it).getId() << " " << (*it).getNumberOfIntervals()<<" ";
		for(vector<double>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2){
			os << *it2 << " ";
		}
		os << endl;
	}


}

} // namespace ILP
