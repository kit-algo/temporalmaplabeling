#ifndef EVALUATOR_H
#define EVALUATOR_H

#include "../heuristics/heuristic.h"

class Evaluator {
public:
  Evaluator(SelectionIntervals *labelIntervals, const char *name);
  void print();
  double getTotalDisplayTime();

private:
  SelectionIntervals *labelIntervals;
  const char *name;
};

#endif
