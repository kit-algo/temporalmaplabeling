#include "utils.h"

#include "config.h"

#include <QCoreApplication>

#include <vector>
#include <cassert>
#include <iostream>

/**
 * The end point of an interval.
 */
class IntervalEnd{

public:
    enum Type{Start,End};
    int id() const {return id_;}
    Type type() const {return static_cast<Type>(type_);}
    double pos() const {return pos_;}

    IntervalEnd(int id, char type, double pos) : id_(id), type_(type), pos_(pos){

    }

    /**
     * Used for sorting the interval ends from left to right. If two interval ends are the same,
     * then the end lies to the left of the start. If both ends have the same type, the id decides.
     */
    bool operator < (const IntervalEnd & ie) const{
       return pos() < ie.pos() || (pos() == ie.pos() && type() == End && ie.type() == Start) ||
              (pos() == ie.pos() && type() == ie.type()  && id() < ie.id());
    }

private:
    int id_ = -1; // the id of the interval
    char type_ =0; // 1: the beginning of the interval, 2: the end of the interval
    double pos_ =-1; // position of the interval;

};

template<class T>
class SubSet{
private:
    const std::vector<T> & container_;
    std::vector<int> indices;

public:
    SubSet(const std::vector<T> & container) :container_(container) {}

    void push_back(int index){
        assert(0  <= index && index < container_.size());
        indices.push_back(index);
    }
    size_t size() const {return indices.size();}

    const T & operator [] (int index) const{return container_[indices[index]];}
};

typedef SubSet<IntervalEnd> EventSet;



std::vector<WeightedInterval> WeightedInterval::substract(const std::vector<WeightedInterval> &intervals)
{
  std::vector<WeightedInterval> result;

  if(intervals.empty()){
      result.push_back(WeightedInterval(id(),start(),end(),weight()));
      return result;
  }

  if(start() < intervals.front().start()){
      result.push_back(WeightedInterval(id(),start(),intervals.front().start(),intervals.front().start()-start()));
  }

  for(size_t i=1; i < intervals.size(); ++i){

      const WeightedInterval & sub1 = intervals[i-1];
      const WeightedInterval & sub2 = intervals[i];
      assert(sub1.end()<= sub2.start());
      WeightedInterval interval(id(),sub1.end(),sub2.start(),sub2.start()-sub1.end());
      if(interval.length() > 0){
          result.push_back(interval);
      }
  }

  if(intervals.back().end() < end()){
      result.push_back(WeightedInterval(id(),intervals.back().end(),end(),end()-intervals.back().end()));
  }

  return result;
}



std::vector<int> computeMWIS(std::vector<IntervalEnd> &startEvents,
                             std::vector<IntervalEnd> &endEvents, const std::vector<double> &weights){



    std::vector<double> table;
    std::vector<int>    predecessors;
    std::vector<int>    last;
    table.resize(endEvents.size()+1,0);
    last.resize(weights.size(),0);
    predecessors.resize(endEvents.size()+1,0);

    size_t s= 1;
    size_t e= 1;
    assert(!startEvents.empty() && !endEvents.empty());
    assert(startEvents.front().pos() <= endEvents.front().pos());
    assert(startEvents.back().pos() <= endEvents.back().pos());
    assert(startEvents.size() == endEvents.size());

    int lastEnd =0;
    while(e < endEvents.size()+1){

        if(s < startEvents.size()+1 && startEvents[s-1] < endEvents[e-1]){ // handle start event
            assert(s < startEvents.size()+1);
            assert(s >= 0);
            IntervalEnd & start = startEvents[s-1];

            last[start.id()]  = lastEnd;
            ++s;
        }else{ // handle end event
            IntervalEnd & end   = endEvents[e-1];

            double weight = weights[end.id()];
            if(table[e-1] > weight+table[last[end.id()]]){
                table[e] = table[e-1];
                predecessors[e] = e-1;
            }else{
                table[e] = table[last[end.id()]]+weight;
                predecessors[e] = last[end.id()];
            }
            lastEnd = e;
            ++e;
        }
    }



    std::vector<int> result;
    int entry = endEvents.size();
    while(entry != 0){
        const IntervalEnd & event = endEvents[entry-1];
        if(predecessors[entry] == last[event.id()]){ //take those intervals that fit.
            result.push_back(event.id());
        }
        entry = predecessors[entry];
    }

    return result;
}

std::vector<int> computeMWIS(std::vector<WeightedInterval> &intervals){
    std::vector<IntervalEnd> endEvents, startEvents;
    std::vector<double> weights;
    for(size_t i=0; i < intervals.size(); ++i){
        const WeightedInterval & wi = intervals[i];
        assert(wi.start() <= wi.end());
        startEvents.push_back(IntervalEnd(i,IntervalEnd::Start,wi.start()));
        endEvents.push_back(IntervalEnd(i,IntervalEnd::End,wi.end()));
        weights.push_back(wi.weight());
    }
    std::sort(endEvents.begin(),endEvents.end());
    std::sort(startEvents.begin(),startEvents.end());

    return computeMWIS(startEvents,endEvents,weights);

}

double check(const std::vector<WeightedInterval> & intervals, const std::vector<int> & mwis);

bool areIndependent(const std::vector<WeightedInterval> & intervals, const std::vector<int> & set){
    std::vector<WeightedInterval> temp;
    for(int index : set){
        temp.push_back(intervals[index]);
    }
    std::sort(temp.begin(), temp.end(),[](const WeightedInterval & wi1, const WeightedInterval & wi2){
        return wi1.start() < wi2.start();
    });
    for(size_t i =1; i  < temp.size(); ++i){
        if(temp[i].start() < temp[i-1].end()){
            std::cout << "check failed at index " << i << std::endl;
            return false;
        }
    }
    return true;
}

void test1(){
    std::vector<WeightedInterval> intervals;


    intervals.push_back(WeightedInterval(0,1,1));  // 0
    intervals.push_back(WeightedInterval(2,3,1));  // 1
    intervals.push_back(WeightedInterval(4,5,1));  // 2
    intervals.push_back(WeightedInterval(6,7,1));  // 3
    intervals.push_back(WeightedInterval(8,9,1));  // 4
    intervals.push_back(WeightedInterval(10,11,1)); // 5

    computeMWIS(intervals);


    assert(check(intervals,computeMWIS(intervals)) == 6);
}

void test2(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(2,3,1));  // 1
    intervals.push_back(WeightedInterval(4,5,1));  // 2
    intervals.push_back(WeightedInterval(6,7,1));  // 3
    intervals.push_back(WeightedInterval(8,9,1));  // 4
    intervals.push_back(WeightedInterval(0,10,1)); // 5

    computeMWIS(intervals);


    assert(check(intervals,computeMWIS(intervals)) == 4);
}


void test3(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(2,3,1));  // 1
    intervals.push_back(WeightedInterval(4,5,1));  // 2
    intervals.push_back(WeightedInterval(6,7,1));  // 3
    intervals.push_back(WeightedInterval(8,9,1));  // 4
    intervals.push_back(WeightedInterval(0,10,5)); // 5

    assert(check(intervals,computeMWIS(intervals)) == 5);
}


void test4(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(0,1,1));  // 1
    intervals.push_back(WeightedInterval(0,2,1));  // 2
    intervals.push_back(WeightedInterval(0,3,1));  // 3
    intervals.push_back(WeightedInterval(0,4,1));  // 4
    intervals.push_back(WeightedInterval(0,5,1)); // 5

    assert(check(intervals,computeMWIS(intervals)) == 1);
}



void test5(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(0,1,1));  // 1
    intervals.push_back(WeightedInterval(1,2,1));  // 2
    intervals.push_back(WeightedInterval(2,3,1));  // 3
    intervals.push_back(WeightedInterval(4,5,1));  // 4
    intervals.push_back(WeightedInterval(6,7,1)); // 5

    assert(check(intervals,computeMWIS(intervals)) == 5);
}


void test6(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(0,1,1));  // 0
    intervals.push_back(WeightedInterval(1,2,1));  // 1
    intervals.push_back(WeightedInterval(2,3,1));  // 2
    intervals.push_back(WeightedInterval(0.5,2.5,1));  // 3
    intervals.push_back(WeightedInterval(2.8,7,1)); // 4

    assert(check(intervals,computeMWIS(intervals)) == 3);
}



void test7(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(0,1,1));  // 0
    intervals.push_back(WeightedInterval(1,2,1));  // 1
    intervals.push_back(WeightedInterval(2,3,1));  // 2
    intervals.push_back(WeightedInterval(0.5,2.5,1));  // 3
    intervals.push_back(WeightedInterval(2.8,7,0.5)); // 4

    assert(check(intervals,computeMWIS(intervals)) == 3);
}


void test8(){
    std::vector<WeightedInterval> intervals;

    intervals.push_back(WeightedInterval(0,1,1));  // 0
    intervals.push_back(WeightedInterval(1,2,1));  // 1
    intervals.push_back(WeightedInterval(0.5,1.5,10));  // 2
    intervals.push_back(WeightedInterval(0.5,1.5,10));  // 3


    assert(check(intervals,computeMWIS(intervals)) == 10);
}

void test9(){
    IntervalEnd e(429,IntervalEnd::End,4799.943587124837);
    IntervalEnd s(374,IntervalEnd::Start,4799.943587124837);
    assert(s < e);
}


double check(const std::vector<WeightedInterval> & intervals, const std::vector<int> & mwis){
   double weight =0;
   std::cout << "[";
   for(size_t i=0; i < mwis.size(); ++i){
       int index = mwis[i];
       std::cout<<index<< (i==mwis.size()-1 ? "" :",");
       weight += intervals[index].weight();

       if(i > 0){
            assert(intervals[mwis[i]].end() <= intervals[mwis[i-1]].start());
       }
   }

   areIndependent(intervals,mwis);

   std::cout << "], ";
   std::cout << "weight=" << weight << std::endl;
   return weight;
}

bool testMWISAlgo(){
#ifdef CONSISTENCY_CHECKS
    std::cout << "Run tests on algorithm for computing weight independent set on interval graphs." << std::endl;
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    test7();
    test8();
   // test9();
    std::cout << "Tests have been passed" << std::endl;
#endif
    return true;
}

// runs the test directly after starting the program.
bool run = testMWISAlgo();
