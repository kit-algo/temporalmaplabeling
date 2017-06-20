#ifndef CLIRUNNER_H
#define CLIRUNNER_H

#include <QObject>
#include <random>

#include "map/map.h"
#include "map/trajectoryfactory.h"

class CLIRunner : public QObject
{
    Q_OBJECT
public:
    CLIRunner(Map *map, int seed, int iterations, const char *graphOutFile, const char *outputFile, const char *ilpOutputFile = nullptr, const char *intervalOutputFile = nullptr, bool fixseed = false, int k = -1);

public Q_SLOTS:
    void run();

Q_SIGNALS:
    void finished();

private:
    Map *map;
    int seed;
    int iterations;
    std::mt19937 rng;
    const char *graphOutFile;
    const char *outputFile;
    const char *ilpOutputFile;
    const char *intervalOutputFile;
    bool fixseed;
    int k;
};

#endif // CLIRUNNER_H
