/*
 * Conflict.cpp
 *
 *  Created on: Jan 10, 2013
 *      Author: benjamin
 */

#include "Conflict.h"

namespace ILP {

Conflict::Conflict(int label1, int label2) {
	this->label1 = label1;
	this->label2 = label2;

}

Conflict::~Conflict() {
	// TODO Auto-generated destructor stub
}

} // namespace ilp
