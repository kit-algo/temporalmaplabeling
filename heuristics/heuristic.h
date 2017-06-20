#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "../conflicts/trajectoryfilter.h"
#include "../util/setrtree.h"

typedef SetRTree<POI *> SelectionIntervals;

class Heuristic {
public:
  virtual SelectionIntervals *getLabelIntervals() = 0;

  enum ModelType {AM1, AM2, AM3};
};

#endif
