/*
 * Conflict.h
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#ifndef CONFLICT_H_
#define CONFLICT_H_

namespace ILP {
using namespace ILP;

class Conflict {

public:

	     int label1;
	     int label2;

	Conflict(int label1, int label2);
	virtual ~Conflict();
	int getLabel1() const {
		return label1;
	}

	int getLabel2() const {
		return label2;
	}

	bool involves(int id){
		return label1 == id || label2 == id;
	}

	int getOpponent(int id){
		return id == label1 ? label2 : (id == label2 ? label1 : -1);
	}

}

;

} // namespace ILP

#endif /* CONFLICT_H_ */
