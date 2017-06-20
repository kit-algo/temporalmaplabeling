/*
 * Import.h
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#ifndef IMPORT_H_
#define IMPORT_H_

//#include <vector>
#include <map>
#include "Intervals.h"
#include "Conflict.h"
#include "Instance.h"

using namespace std;
using namespace ILP;

namespace ILP {

class Import {

private:

	Intervals * readIntervals(string line);
	Conflict  * readConflict(string line);

public:
	Import();
	virtual ~Import();

	void import(string  file, Instance* instance);

	void exportIntervals(string file, vector<Intervals> & intervals);


};

} // namespace ILP

#endif /* IMPORT_H_ */
