#include "evaluator.h"

Evaluator::Evaluator(SelectionIntervals *labelIntervals, const char *name)
  : labelIntervals(labelIntervals), name(name)
{
}

void
Evaluator::print()
{
  std::cout << "===== Evaluation =====\n";
  std::cout << "Name: \t\t\t" << this->name << "\n";
  std::cout << "Cumulative Display Distance:\t" << this->getTotalDisplayTime() << "\n";
  std::cout << "======================\n";
}

double
Evaluator::getTotalDisplayTime()
{
  double total = 0.0;

  for (auto entry : this->labelIntervals->queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
    Interval interval = entry.first;
    double dist = (bg::get<0>(interval.second) - bg::get<0>(interval.first));
    total += dist;
  }

  return total;
}
