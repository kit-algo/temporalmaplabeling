#include "clirunner.h"

#include "tests/conflicttest.h"

#include "../ilp/adapter.h"
#include "../heuristics/greedyheuristic.h"
#include "../heuristics/intervalgraphheuristic.h"
#include "conflicts/conflictgraph.h"

#include "checks/resultconsistencychecker.h"
#include "checks/nooverlapschecker.h"

#include "evaluator.h"
#include "config.h"

#include <assert.h>
#include <QDebug>
#include <sstream>
#include <string>
#include <fstream>
#include <ios>

CLIRunner::CLIRunner(Map *map, int seed, int iterations, const char *graphOutFile, const char *outputFile, const char *ilpOutputFile, const char *intervalOutputFile, bool fixseed, int k):
    map(map), seed(seed), iterations(iterations), graphOutFile(graphOutFile), outputFile(outputFile), ilpOutputFile(ilpOutputFile), intervalOutputFile(intervalOutputFile), fixseed(fixseed), k(k)
{
    this->rng = std::mt19937(seed);
}

void CLIRunner::run()
{
  std::cout << "Running CLI";

/*
  ConflictTest ct(this->map, this->seed);
  ct.run(5);
*/

  std::uniform_int_distribution<int> uni(0, std::numeric_limits<int>::max());

  std::ofstream outfile;
  outfile.open(this->outputFile, std::ios::out );
  outfile << "FILE-VERSION," << "K," << "SEED";
  outfile << "," << "ILP AM1 SCORE" << "," << "ILP AM2 SCORE" << "," << "ILP AM3 SCORE" << "," << "ILP AM1 BOUND" << "," << "ILP AM2 BOUND" << "," << "ILP AM3 BOUND" << "," << "ILP AM1 GAP " << "," << "ILP AM2 GAP " << "," << "ILP AM3 GAP " << "," << "ILP AM1 TIME" << "," << "ILP AM2 TIME" << "," << "ILP AM3 TIME";
  outfile  << "," << "GREEDY AM1 SCORE" << "," << "GREEDY AM1 COMPOUND SCORE" << "," << "GREEDY AM2 SCORE" << "," << "GREEDY AM3 SCORE" << "," << "GREEDY AM1 TIME" << "," << "GREEDY AM1 COMPOUND TIME" << "," << "GREEDY AM2 TIME" << "," << "GREEDY AM3 TIME";
  outfile  << "," << "INTGRAPH AM1 SCORE" << "," << "INTGRAPH AM2 SCORE" << "," << "INTGRAPH AM3 SCORE" << "," << "INTGRAPH AM1 TIME" << "," << "INTGRAPH AM2 TIME" << "," << "INTGRAPH AM3 TIME";
  outfile << ", ROTATIONAL CONFLICT TIME" << ", ZOOMING CONFLICT TIME" << ", PATH CREATION TIME" << ", INTERPOLATION TIME" << ", GRAPH AM1 TIME" << ", GRAPH AM2 TIME" << ", GRAPH AM3 TIME";

  outfile << ", NUMBER OF CONFLICTS" << ", NUMBER OF VISIBILITIES " << ", TRAJECTORY LENGTH" << ", TRAJECTORY SIZE" << ", VISIBILITIES LENGTH" << ", CONFLICTS LENGTH" ;
  outfile << ", AM1 GRAPH NODES" << ", AM1 GRAPH EDGES" << ", AM2 GRAPH NODES" << ", AM2 GRAPH EDGES" << ", AM3 GRAPH NODES" << ", AM3 GRAPH EDGES";

  outfile << "\n";

  for (int i = 0 ; i < iterations ; i++) {
    int cur_seed = uni(this->rng);
    if (fixseed) {
      cur_seed = this->seed;
    }
    Router router(this->map, cur_seed);

    // Write instance to outfile first, in case of a crash...
    outfile << "6," << this->k << "," << cur_seed;
    outfile << std::flush;

    std::vector<std::pair<edge_t, bool>> route = router.get_random_route();
    std::vector<Position> route_waypoints;
    std::vector<double> route_speeds;

    for (auto edge : route) {
        std::vector<Position> waypoints = this->map->get_waypoints(edge.first);
        route_waypoints.reserve(route_waypoints.size() + waypoints.size());
        route_speeds.insert(route_speeds.end(), waypoints.size(), this->map->get_way_speed(edge.first));

        if (edge.second) { // Traversing in edge direction..
            route_waypoints.insert(route_waypoints.end(), waypoints.begin(), waypoints.end());
        } else { // Traversing against edge direction
            route_waypoints.insert(route_waypoints.end(), waypoints.rbegin(), waypoints.rend());
        }
    }

    Camera *camera = NULL;
    Trajectory *trajectory = NULL;
    try {
      TrajectoryFactory tFac(&route_waypoints, &route_speeds, CAMERA_WIDTH, CAMERA_HEIGHT, this->map);
      trajectory = tFac.getTrajectory();

      std::cout << "Trajectory length: " << trajectory->getLength() << "\n";
      camera = new Camera(CAMERA_WIDTH, CAMERA_HEIGHT, trajectory, this->map, LABEL_FONT);
      std::cout << "Camera created...\n";
      camera->compute();
    } catch (char const *err) {
      std::cout << "!!!!!!!!!!!! Exception !!!!!!!!!!!!\n";
      std::cout << err << "\n";
      return;
    }


#ifdef OMGCHECK
    NoOverlapsChecker noc(camera, map, trajectory);
    assert(noc.check());
#endif

    /* Testing the various heuristics / the ILP */

    std::cout << "Total conflicts computed: " << camera->getConflictIntervals().size() << "\n";
    double totalConflictLength = 0;
    for (auto entry : camera->getConflictIntervals().queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
      Interval interval = entry.first;
      double dist = (bg::get<0>(interval.second) - bg::get<0>(interval.first));
      totalConflictLength += dist;
    }

    double totalVisibilityLength = 0;
    for (auto entry : camera->getVisibilityIntervals().queryFull(bgi::satisfies([](Interval const&){ return true; }))) {
      Interval interval = entry.first;
      double dist = (bg::get<0>(interval.second) - bg::get<0>(interval.first));
      totalVisibilityLength += dist;
    }

    /* Greedy Heuristic AM1! */
    //std::cout << "Building conflict graph only-AM1...\n";
  //  ConflictGraph *cg = ConflictGraph::fromConflicts(camera->getConflictIntervals(), camera->getVisibilityIntervals());

    std::cout << "Building AM 1 conflict graph...\n";
    double graph_am1_time = -1;
    Clock graph_am1_clock;
    graph_am1_clock.start();
    ExpandedConflictGraph *ecg_am1 = ExpandedConflictGraph::fromConflicts(camera->getConflictIntervals(), camera->getVisibilityIntervals(), Heuristic::AM1, false);
    if (ecg_am1 != nullptr) {
      graph_am1_time = graph_am1_clock.stop();
    }

    std::cout << "Building AM 1 compound conflict graph...\n";
    ExpandedConflictGraph *ecg_am1_comp = ExpandedConflictGraph::fromConflicts(camera->getConflictIntervals(), camera->getVisibilityIntervals(), Heuristic::AM1, true);

    std::cout << "Building AM 2 conflict graph...\n";
    double graph_am2_time = -1;
    Clock graph_am2_clock;
    graph_am2_clock.start();
    ExpandedConflictGraph *ecg_am2 = ExpandedConflictGraph::fromConflicts(camera->getConflictIntervals(), camera->getVisibilityIntervals(), Heuristic::AM2, false);
    if (ecg_am2 != nullptr) {
      graph_am2_time = graph_am2_clock.stop();
    }

    std::cout << "Building AM 3 conflict graph...\n";
    double graph_am3_time = -1;
    Clock graph_am3_clock;
    graph_am3_clock.start();
    ExpandedConflictGraph *ecg_am3 = ExpandedConflictGraph::fromConflicts(camera->getConflictIntervals(), camera->getVisibilityIntervals(), Heuristic::AM3, false);
    if (ecg_am3 != nullptr) {
      graph_am3_time = graph_am3_clock.stop();
    }

    if (this->intervalOutputFile != nullptr) {
      ostringstream conflicts_filename;
      conflicts_filename << this->intervalOutputFile << "-seed" << cur_seed << "-conflicts.csv";
      camera->getConflictIntervals().write(conflicts_filename.str().c_str());


      ostringstream visibilities_filename;
      visibilities_filename << this->intervalOutputFile << "-seed" << cur_seed << "-visibilities.csv";
      camera->getVisibilityIntervals().write(visibilities_filename.str().c_str());
    }

    if (this->graphOutFile != nullptr) {
      // ostringstream am_1_filename_legacy;
      // am_1_filename_legacy << this->graphOutFile << "-seed" << cur_seed << "-AM1-legacy.graphml";
      // ConflictGraph::writeToFile(am_1_filename_legacy.str().c_str(), &cg);

      ostringstream am_1_filename;
      am_1_filename << this->graphOutFile << "-seed" << cur_seed << "-AM1.graphml";
      if (ecg_am1 != nullptr)
        ExpandedConflictGraph::writeToFile(am_1_filename.str().c_str(), *ecg_am1);

      ostringstream am_2_filename;
      am_2_filename << this->graphOutFile << "-seed" << cur_seed << "-AM2.graphml";
      if (ecg_am2 != nullptr)
        ExpandedConflictGraph::writeToFile(am_2_filename.str().c_str(), *ecg_am2);

      ostringstream am_3_filename;
      am_3_filename << this->graphOutFile << "-seed" << cur_seed << "-AM3.graphml";
      if (ecg_am3 != nullptr)
        ExpandedConflictGraph::writeToFile(am_3_filename.str().c_str(), *ecg_am3);
    }



    std::cout << "Running AM1 ig heuristic..." << std::endl;
    double igh_am1_score = -1;
    double igh_am1_time = -1;
    if (ecg_am1 != nullptr) {
      Clock igh_clock;

      IGHeuristic ighAM1(*ecg_am1,IGHeuristic::AM1, this->k);
      igh_clock.start();
      ighAM1.run();
      igh_am1_time = igh_clock.stop();

      Evaluator ighAM1Eval(ighAM1.getLabelIntervals(), "Interval Graph AM1");
      igh_am1_score = ighAM1Eval.getTotalDisplayTime();

      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-IGH-AM1.csv";
        ighAM1.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }
#ifndef NDEBUG
    std::cout << "score: " << igh_am1_score  << std::endl;
    std::cout << "time: " << igh_am1_time  << std::endl;
#endif


    std::cout << "Running AM2 ig heuristic..." << std::endl;
    double igh_am2_score = -1;
    double igh_am2_time = -1;
    if (ecg_am1 != nullptr) {
      Clock igh_clock;

      IGHeuristic ighAM2(*ecg_am1,IGHeuristic::AM2, this->k);
         igh_clock.start();
      ighAM2.run();
      igh_am2_time = igh_clock.stop();

      Evaluator ighAM2Eval(ighAM2.getLabelIntervals(), "Interval Graph AM2");
      igh_am2_score = ighAM2Eval.getTotalDisplayTime();

      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-IGH-AM2.csv";
        ighAM2.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }
#ifndef NDEBUG
    std::cout << "score: " << igh_am2_score  << std::endl;
    std::cout << "time: " << igh_am2_time  << std::endl;
#endif

    std::cout << "Running AM3 ig heuristic..." << std::endl;
    double igh_am3_score = -1;
    double igh_am3_time = -1;
    if (ecg_am1 != nullptr) {
      Clock igh_clock;

      IGHeuristic ighAM3(*ecg_am1,IGHeuristic::AM3, this->k);
      igh_clock.start();
      ighAM3.run();
      igh_am3_time = igh_clock.stop();

      Evaluator ighAM3Eval(ighAM3.getLabelIntervals(), "Interval Graph AM3");
      igh_am3_score = ighAM3Eval.getTotalDisplayTime();

      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-IGH-AM3.csv";
        ighAM3.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }
#ifndef NDEBUG
    std::cout << "score: " << igh_am3_score  << std::endl;
    std::cout << "time: " << igh_am3_time  << std::endl;
#endif

    camera->getVisibilityIntervals().write("/tmp/visibilities-after-ig-heuristics.csv");


    std::cout << "Running AM1 greedy heuristic..."<<std::endl;
    double gh_am1_score = -1;
    double gh_am1_time = -1;
    if (ecg_am1 != nullptr) {
      Clock gh_clock;

      GreedyHeuristic ghAM1(*ecg_am1, this->k);
       gh_clock.start();
      ghAM1.run();
      gh_am1_time = gh_clock.stop();

      Evaluator ghAM1Eval(ghAM1.getLabelIntervals(), "Greedy AM1");
      gh_am1_score = ghAM1Eval.getTotalDisplayTime();

#ifdef CONSISTENCY_CHECKS
      std::cout << "Running consistency checks...\n";
      ResultConsistencyChecker rcc(ghAM1.getLabelIntervals(), camera, map, this->k);
      rcc.check();
#endif

      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-GH-AM1.csv";
        ghAM1.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }

#ifndef NDEBUG
    std::cout << "score: " << gh_am1_score  << std::endl;
    std::cout << "time: " << gh_am1_time  << std::endl;
#endif


    std::cout << "Running AM1 compound greedy heuristic..."<<std::endl;
    double gh_am1_comp_score = -1;
    double gh_am1_comp_time = -1;
    if (ecg_am1_comp != nullptr) {
      Clock gh_clock;

      GreedyHeuristic ghAM1(*ecg_am1_comp, this->k);
      gh_clock.start();
      ghAM1.run();
      gh_am1_comp_time = gh_clock.stop();

      Evaluator ghAM1Eval(ghAM1.getLabelIntervals(), "Greedy AM1 Compound");
      gh_am1_comp_score = ghAM1Eval.getTotalDisplayTime();

    #ifdef CONSISTENCY_CHECKS
      std::cout << "Running consistency checks...\n";
      ResultConsistencyChecker rcc(ghAM1.getLabelIntervals(), camera, map, this->k);
      rcc.check();
    #endif

      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-GH-AM1-comp.csv";
        ghAM1.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }


    double gh_am2_score = -1;
    double gh_am2_time = -1;
    std::cout << "Running AM2 greedy heuristic...\n";
    if (ecg_am2 != nullptr) {
      Clock gh_clock;

      GreedyHeuristic ghAM2(*ecg_am2, this->k);
      gh_clock.start();
      ghAM2.run();
      gh_am2_time = gh_clock.stop();

      Evaluator ghAM2Eval(ghAM2.getLabelIntervals(), "Greedy AM2");
      gh_am2_score = ghAM2Eval.getTotalDisplayTime();

#ifdef CONSISTENCY_CHECKS
      std::cout << "Running consistency checks...\n";
      ResultConsistencyChecker rcc(ghAM2.getLabelIntervals(), camera, map, this->k);
      rcc.check();
#endif


      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-GH-AM2.csv";
        ghAM2.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }

#ifndef NDEBUG
    std::cout << "score: " << gh_am2_score  << std::endl;
    std::cout << "time: " << gh_am2_time  << std::endl;
#endif



    double gh_am3_score = -1;
    double gh_am3_time = -1;
    std::cout << "Running AM3 greedy heuristic...\n";
    if (ecg_am3 != nullptr) {
      Clock gh_clock;

      GreedyHeuristic ghAM3(*ecg_am3, this->k);
      gh_clock.start();
      ghAM3.run();
      gh_am3_time = gh_clock.stop();

      Evaluator ghAM3Eval(ghAM3.getLabelIntervals(), "Greedy AM3");
      gh_am3_score = ghAM3Eval.getTotalDisplayTime();

#ifdef CONSISTENCY_CHECKS
      std::cout << "Running consistency checks...\n";
      ResultConsistencyChecker rcc(ghAM3.getLabelIntervals(), camera, map, this->k);
      rcc.check();
#endif


      if (this->intervalOutputFile != nullptr) {
        ostringstream selection_filename;
        selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-GH-AM3.csv";
        ghAM3.getLabelIntervals()->write(selection_filename.str().c_str());
      }
    }

#ifndef NDEBUG
    std::cout << "score: " << gh_am3_score  << std::endl;
    std::cout << "time: " << gh_am3_time  << std::endl;
#endif

    camera->getVisibilityIntervals().write("/tmp/visibilities-after-greedy-heuristics.csv");


    // Graph Complexity measures
    long edges_am1 = -1, edges_am2 = -1, edges_am3 = -1, vertices_am1 = -1, vertices_am2 = -1, vertices_am3 = -1;
    if (ecg_am1 != nullptr) {
      edges_am1 = boost::num_edges(*ecg_am1);
      vertices_am1 = boost::num_vertices(*ecg_am1);
    }

    if (ecg_am2 != nullptr) {
      edges_am2 = boost::num_edges(*ecg_am2);
      vertices_am2 = boost::num_vertices(*ecg_am2);
    }

    if (ecg_am3 != nullptr) {
      edges_am3 = boost::num_edges(*ecg_am3);
      vertices_am3 = boost::num_vertices(*ecg_am3);
    }

    if (ecg_am1 != nullptr)
      delete ecg_am1;
    if (ecg_am2 != nullptr)
      delete ecg_am2;
    if (ecg_am3 != nullptr)
      delete ecg_am3;

    /* First: The ILP, AM1 */
    std::cout << "Running AM1 ILP...\n";
    double ilp_AM1_score;
    double ilp_AM1_bound;
    double ilp_AM1_gap;

    double ilp_AM1_time = -1;
    if (COMPUTE_LARGE_ILPS || (ecg_am1 != nullptr)) {
      const char *ilp_filename;
      string buf;
      if (this->ilpOutputFile != nullptr) {
        ostringstream ilp_filename_stream;
        ilp_filename_stream << this->ilpOutputFile << "-AM1.mps";
        buf = ilp_filename_stream.str();
        ilp_filename = buf.c_str();
      } else {
        ilp_filename = nullptr;
      }

      ILPAdapter ilp1(this->map, camera->getVisibilityIntervals(), camera->getConflictIntervals(), Heuristic::AM1, this->k, ilp_filename);
      Clock ilp_clock;
      ilp_clock.start();
      ilp1.run();
      double tmp_time = ilp_clock.stop();
      ilp_AM1_bound = ilp1.getBound();
      ilp_AM1_gap = ilp1.getGap();

      if (ilp1.getLabelIntervals() != nullptr) {
        Evaluator ilpAM1Eval(ilp1.getLabelIntervals(), "ILP AM1");
        ilp_AM1_score = ilpAM1Eval.getTotalDisplayTime();
        ilpAM1Eval.print();
        ilp_AM1_time = tmp_time;

        if (this->intervalOutputFile != nullptr) {
          ostringstream selection_filename;
          selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-ILP-AM1.csv";
          ilp1.getLabelIntervals()->write(selection_filename.str().c_str());
        }

#ifdef CONSISTENCY_CHECKS
        std::cout << "Running consistency checks...\n";
        ResultConsistencyChecker rcc(ilp1.getLabelIntervals(), camera, map, this->k);
        rcc.check();
#endif

      } else {
        ilp_AM1_score = -2;
      }
    } else {
      ilp_AM1_score = -1;
    }

    camera->getVisibilityIntervals().write("/tmp/visibilities-after-am1-ilp.csv");


    /* The ILP, AM2 */
    std::cout << "Running AM2 ILP...\n";

    double ilp_AM2_score;
    double ilp_AM2_bound;
    double ilp_AM2_gap;

    double ilp_AM2_time = -1;
    if (COMPUTE_LARGE_ILPS || (ecg_am2 != nullptr)) {
      const char *ilp_filename;
      string buf;
      if (this->ilpOutputFile != nullptr) {
        ostringstream ilp_filename_stream;
        ilp_filename_stream << this->ilpOutputFile << "-AM2.mps";
        buf = ilp_filename_stream.str();
        ilp_filename = buf.c_str();
      } else {
        ilp_filename = nullptr;
      }

      ILPAdapter ilp2(this->map, camera->getVisibilityIntervals(), camera->getConflictIntervals(), Heuristic::AM2, this->k, ilp_filename);

      Clock ilp_clock;
      ilp_clock.start();
      ilp2.run();
      double tmp_time = ilp_clock.stop();

      ilp_AM2_bound = ilp2.getBound();
      ilp_AM2_gap = ilp2.getGap();

      if (ilp2.getLabelIntervals() != nullptr) {
        Evaluator ilpAM2Eval(ilp2.getLabelIntervals(), "ILP AM2");
        ilp_AM2_score = ilpAM2Eval.getTotalDisplayTime();
        ilpAM2Eval.print();
        ilp_AM2_time = tmp_time;

        if (this->intervalOutputFile != nullptr) {
          ostringstream selection_filename;
          selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-ILP-AM2.csv";
          ilp2.getLabelIntervals()->write(selection_filename.str().c_str());
        }
      } else {
        ilp_AM2_score = -2;
      }
    } else {
      ilp_AM2_score = -1;
    }

    /* The ILP, AM3 */
    std::cout << "Running AM3 ILP...\n";

    double ilp_AM3_score;
    double ilp_AM3_bound;
    double ilp_AM3_gap;

    double ilp_AM3_time = -1;
    if (COMPUTE_LARGE_ILPS || (ecg_am3 != nullptr)) {
      const char *ilp_filename;
      string buf;
      if (this->ilpOutputFile != nullptr) {
        ostringstream ilp_filename_stream;
        ilp_filename_stream << this->ilpOutputFile << "-AM3.mps";
        buf = ilp_filename_stream.str();
        ilp_filename = buf.c_str();
      } else {
        ilp_filename = nullptr;
      }

      ILPAdapter ilp3(this->map, camera->getVisibilityIntervals(), camera->getConflictIntervals(), Heuristic::AM3, this->k, ilp_filename);

      Clock ilp_clock;
      ilp_clock.start();
      ilp3.run();
      double tmp_time = ilp_clock.stop();

      ilp_AM3_bound = ilp3.getBound();
      ilp_AM3_gap = ilp3.getGap();

      if (ilp3.getLabelIntervals() != nullptr) {
        Evaluator ilpAM3Eval(ilp3.getLabelIntervals(), "ILP AM3");
        ilp_AM3_score = ilpAM3Eval.getTotalDisplayTime();
        ilpAM3Eval.print();

        ilp_AM3_time = tmp_time;

        if (this->intervalOutputFile != nullptr) {
          ostringstream selection_filename;
          selection_filename << this->intervalOutputFile << "-seed" << cur_seed << "-selection-ILP-AM3.csv";
          ilp3.getLabelIntervals()->write(selection_filename.str().c_str());
        }
      } else {
        ilp_AM3_score = -2;
      }
    } else {
      ilp_AM3_score = -1;
    }

    //ghEval.print();

    // print CSV
    // ILP Scores
    outfile << "," << ilp_AM1_score << "," << ilp_AM2_score << "," << ilp_AM3_score;
    // ILP Bounds
    outfile << "," << ilp_AM1_bound << "," << ilp_AM2_bound << "," << ilp_AM3_bound;
    // ILP Gaps
    outfile << "," << ilp_AM1_gap << "," << ilp_AM2_gap << "," << ilp_AM3_gap;
    // ILP Timings
    outfile << "," << ilp_AM1_time << "," << ilp_AM2_time << "," << ilp_AM3_time;

    // Greedy Scores
    outfile  << "," << gh_am1_score << "," << gh_am1_comp_score << "," << gh_am2_score << "," << gh_am3_score;
    // Greedy Timings
    outfile  << "," << gh_am1_time << "," << gh_am1_comp_time << "," << gh_am2_time << "," << gh_am3_time;

    // Interval-Graph-Heuristic Scores
    outfile  << "," << igh_am1_score << "," << igh_am2_score << "," << igh_am3_score;
    // Interval-Graph-Heuristic Timings
    outfile  << "," << igh_am1_time << "," << igh_am2_time << "," << igh_am3_time;


    // General Timings
    outfile << "," << camera->time_rotation_conflicts << "," << camera->time_zoom_conflicts << "," << camera->time_path << "," << camera->time_interpolate;
    // Graph Timings
    outfile << "," << graph_am1_time << "," << graph_am2_time << "," << graph_am3_time;


    // General Complexity measures
    outfile << "," << camera->getConflictIntervals().full_size() << "," << camera->getVisibilityIntervals().full_size() << "," << trajectory->getLength() << "," << trajectory->getItems().size() << "," << totalVisibilityLength << "," << totalConflictLength;

    outfile << "," << vertices_am1 << "," << edges_am1 << "," << vertices_am2 << "," << edges_am2 << "," << vertices_am3 << "," << edges_am3;

    outfile << "\n";
    outfile.flush();

    delete camera;
    delete trajectory;
  }
  outfile.close();
  Q_EMIT finished();
}
