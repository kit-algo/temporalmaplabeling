#include "activitycheck.h"

bool contains(const Interval & i1,const Interval &i2){
    return bg::get<0>(i1.first) <= bg::get<0>(i2.first) &&
            bg::get<0>(i1.second) <= bg::get<0>(i2.second);
}

bool disjoint(const Interval & i1,const Interval & i2){
    return bg::get<0>(i1.second) <= bg::get<0>(i2.first) ||
            bg::get<0>(i2.second) <= bg::get<0>(i1.first);
}

bool intersect(const Interval & i1, const Interval & i2 ){
    return !disjoint(i1,i2);
}


bool checkActivity(SelectionIntervals & activity, ExpandedConflictGraph & g, int model){

//    std::vector<std::pair<Interval, POI*>> intervals =
//    activity.queryFullSorted(bgi::satisfies([](Interval const&){ return true; }));

//    std::map<POI*,std::vector<ExpandedConflictGraph::vertex_t>> vertexMap;


//    auto vs = boost::vertices(g);
//    for (auto vit = vs.first; vit != vs.second; ++vit) {
//         vertexMap[g[*vit].poi].push_back(*vit);
//    }


//    /**
//      Check:
//      - For each activity there is a presence interval that contains the activity interval.
//      - Each presence interval contains at most one activtity interval.
//      */
//    std::map<ExpandedConflictGraph::vertex_t,int> presenceToActivity;
//    std::map<int,ExpandedConflictGraph::vertex_t> activityToPresence;
//    for(std::size_t i=0; i < intervals.size(); ++i){
//         std::pair<Interval, POI*> & pair = intervals[i];
//         bool foundPresence = false;
//         for(ExpandedConflictGraph::vertex_t & v : vertexMap[pair.second]){
//             if(intersect(g[v].interval,pair.first)){
//                if(!contains(g[v].interval,pair.first)){
//                    std::cout << "There is a presence interval intersecting a activity interval, but not containing it" << std::endl;
//                    return false;
//                }
//                if(activityMap.find(v) != activityMap.end()){
//                     std::cout << "There is a presence interval that contains multiple actvity intervals" << std::endl;
//                     return false;
//                }
//                activityMap[v] = i;
//                foundPresence = true;
//             }
//         }
//         if(!foundPresence){
//             std::cout << "There is an activity interval that is not contained in any presence interval"<<std::endl;
//             return false;
//         }
//    }

//    /**
//      If two activity intervals intersect they may not be in conflict.
//      */
//    for(std::size_t i=0; i < intervals.size(); ++i){
//        std::pair<Interval, POI*> & p1 = intervals[i];
//        assert(activityMap.find(i)!= activityMap.end());
//        ExpandedConflictGraph::vertex_t & v1 = activityMap[i];

//        for(std::size_t j=i+1; j < intervals.size(); ++j){
//            assert(activityMap.find(j)!= activityMap.end());
//            ExpandedConflictGraph::vertex_t & v2 = activityMap[i];
//            std::pair<Interval, POI*> & p2 = intervals[j];

//            if(intersect(p1.first,p2.first)){

//                ExpandedConflictGraph::vertex_t & v2 = activityMap[j];
//            }
//        }
//    }

//    return true;
}
