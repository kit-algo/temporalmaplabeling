#include "ui/labelrotation.h"
#include "map/mapreader.h"
#include <QApplication>
#include "app.h"
#include "cli/clirunner.h"
#include "config.h"

#include <time.h>
#include <QTimer>
#include <QDebug>

#include "optionparser.h"

option::ArgStatus ArgMandatory( const option::Option & option, bool) {
  if (option.arg)
    return option::ARG_OK;
  else
    std::cout << "You must specify an argument to the " << option.name << " option\n";
    return option::ARG_ILLEGAL;
}

enum  optionIndex { MAP, PYCGR, SEED, GUI, ITERATIONS, THREADS, GRAPH, OUTPUT, ILPOUT, INTERVALS, FIXSEED, KRESTRICT, SCREENSHOT, SCREENSHOTPOS, SCREENSHOTHEU, SCREENSHOTMODEL, SCREENSHOTK, HELP};
const option::Descriptor usage[] =
{
 {MAP, 0,"m" , "map"    ,ArgMandatory, "Set the OSM map file\n" },
 {PYCGR, 0,"p" , "pycgr"    ,ArgMandatory, "Set the PYCGR map file\n" },
 {OUTPUT, 0,"o" , "out"    ,ArgMandatory, "Output file\n" },
 {SEED,    0,"s" , "seed",ArgMandatory, "Set the seed. YOU SHOULD ALWAYS DO THIS!\n" },
 {KRESTRICT,    0,"k" , "k-restriction",ArgMandatory, "Set the k restriction value\n" },
 {GRAPH, 0,"g" , "graph"    ,ArgMandatory, "Set the graph output file\n" },
 {ILPOUT, 0,"d" , "ilpout"    ,ArgMandatory, "Set the ILP output file\n" },
 {INTERVALS, 0,"v" , "intervals"    ,ArgMandatory, "Set the intervals output file prefix\n" },

 {GUI,    0,"" , "gui",option::Arg::None, "Run in GUI mode\n" },
 {FIXSEED,    0,"f" , "fixseed",option::Arg::None, "Fix to only one seed\n" },

 {THREADS,    0,"t" , "threads",ArgMandatory, "Set the number of threads\n" },
 {ITERATIONS,    0,"i" , "iterations",ArgMandatory, "Set the number of iterations\n" },

 {SCREENSHOT, 0, "", "screenshot"    ,ArgMandatory, "Go into screenshot mode, set screenshot output file\n" },
 {SCREENSHOTHEU, 0, "", "screenshot-heuristic"    ,ArgMandatory, "set screenshot heuristic\n" },
 {SCREENSHOTMODEL, 0, "", "screenshot-model"    ,ArgMandatory, "set screenshot model\n" },
 {SCREENSHOTPOS, 0, "", "screenshot-position"    ,ArgMandatory, "set screenshot position\n" },
 {SCREENSHOTK, 0, "", "screenshot-k"    ,ArgMandatory, "set screenshot k\n" },

 {HELP,    0,"h" , "help",option::Arg::None, "print this help\n" },

 {0,0,0,0,0,0}
};


int main(int argc, char *argv[])
{
  App a(argc, argv);

  int parser_argc = argc;
  auto parser_argv = argv;

  parser_argc-=(parser_argc>0); parser_argv+=(parser_argc>0); // skip program name argv[0] if present
  option::Stats  stats(usage, parser_argc, parser_argv);
  option::Option options[stats.options_max], buffer[stats.buffer_max];
  option::Parser parse(usage, parser_argc, parser_argv, options, buffer);

  if (parse.error())
    return 1;

  if (options[HELP] || argc == 0) {
    option::printUsage(std::cout, usage);
    return 0;
  }


  if (options[MAP].count() != 1) {
    std::cout << "You must specify a map.\n";
  }
  if (options[PYCGR].count() != 1) {
    std::cout << "You must specify a pycgr map.\n";
  }

  if (options[THREADS].count() == 1) {
    num_threads = std::atoi(options[THREADS].arg);
  }


  MapReader mapreader(options[MAP].arg, options[PYCGR].arg);
  mapreader.run();
  Map * map = &(mapreader.get_map());

  //int seed = 4; // chosen by fair xkcd
  int seed = time(nullptr);

  if (options[SEED].count() > 0) {
    seed = std::atoi(options[SEED].arg);
  }

  int k = -1;
  if (options[KRESTRICT].count() > 0) {
    k = std::atoi(options[KRESTRICT].arg);
  }

  std::map<std::string, std::set<std::string>> filter = POI_DEFAULT;
  map->set_poi_filter(filter);


  screenshot_info *sinfo = nullptr;
  if (options[SCREENSHOT].count() > 0) {
    sinfo = new screenshot_info();
    sinfo->filename = options[SCREENSHOT].arg;

    if (options[SCREENSHOTPOS].count() != 1) {
      std::cout << "You must specify a screenshot position.\n";
      exit(-1);
    }
    sinfo->position = atoi(options[SCREENSHOTPOS].arg);


    if (options[SCREENSHOTK].count() != 1) {
      sinfo->k = -1;
    }
    sinfo->k = atoi(options[SCREENSHOTK].arg);

    if (options[SEED].count() != 1) {
      std::cout << "You must specify a screenshot seed.\n";
      exit(-1);
    }
    sinfo->seed = atoi(options[SEED].arg);


    if (options[SCREENSHOTHEU].count() != 1) {
      std::cout << "You must specify a screenshot heuristic.\n";
      exit(-1);
    }
    const char *heuristic = options[SCREENSHOTHEU].arg;


    if (options[SCREENSHOTMODEL].count() != 1) {
      std::cout << "You must specify a screenshot model.\n";
      exit(-1);
    }
    int model = atoi(options[SCREENSHOTMODEL].arg);
    if ((model < 1) || (model > 3)) {
      std::cout << "UNKNOWN MODEL\n";
      exit(-1);
    }

    int tag = (model - 1) * 3;
    if (std::strcmp(heuristic, "ilp") == 0) {
      tag += 0;
    } else if  (std::strcmp(heuristic, "greedy") == 0) {
      tag += 1;
    } else if  (std::strcmp(heuristic, "ig") == 0) {
      tag += 2;
    } else {
      std::cout << "UNKNOWN HEURISTIC\n";
      exit(-1);
    }

    sinfo->heuristic = tag;
  }

  if (options[GUI].count() > 0) {
      std::cout << "Running in GUI mode.";
      LabelRotation w(map, sinfo);
      w.show();
      return a.exec();
  } else {
      std::cout << "Running in CLI mode.";
      int iterations = 1;

      if (options[OUTPUT].count() != 1) {
        std::cout << "You must specify an output file.\n";
      }

      const char *outputFile = options[OUTPUT].arg;

      if (options[ITERATIONS].count() == 1) {
        iterations = std::atoi(options[ITERATIONS].arg);
      }

      const char *graphOutFile = nullptr;
      if (options[GRAPH].count() > 0) {
        graphOutFile = options[GRAPH].arg;
      }

      const char *ilpOutputFile = nullptr;
      if (options[ILPOUT].count() > 0) {
        ilpOutputFile = options[ILPOUT].arg;
      }

      const char *intervalsOutputFile = nullptr;
      if (options[INTERVALS].count() > 0) {
        intervalsOutputFile = options[INTERVALS].arg;
      }

      bool fixseed = false;
      if (options[FIXSEED].count() > 0) {
        fixseed = true;
      }

      CLIRunner *cli = new CLIRunner(map, seed, iterations, graphOutFile, outputFile, ilpOutputFile, intervalsOutputFile, fixseed, k);
      cli->run();
      /*
      QObject::connect(cli, SIGNAL(finished()), &a, SLOT(quit()));
      QTimer::singleShot(0, cli, SLOT(run()));
      */
  }

  //
}
