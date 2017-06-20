#-------------------------------------------------
#
# Project created by QtCreator 2015-08-05T22:35:18
#
#-------------------------------------------------

QT       += core gui svg

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tmlframework
TEMPLATE = app

CONFIG += debug no_keywords

SOURCES += main.cpp\
        ui/labelrotation.cpp \
    map/mapreader.cpp \
    map/map.cpp \
    map/router.cpp \
    ui/drawingcontroller.cpp \
    ui/widgets/sceneview.cpp \
    map/trajectory.cpp \
    map/projection.cpp \
    util/algvec.cpp \
    util/coordinateutil.cpp \
    ui/poifilterchooser.cpp \
    conflicts/rotatingpoi.cpp \
    conflicts/conflict.cpp \
    conflicts/rotatingpoifactory.cpp \
    conflicts/camera.cpp \
    map/trajectoryinterpolator.cpp \
    ui/widgets/cameraview.cpp \
    conflicts/trajectoryfilter.cpp \
    app.cpp \
    cli/clirunner.cpp \
    tests/conflicttest.cpp \
    heuristics/greedyheuristic.cpp \
    ilp/adapter.cpp \
    ilp/Conflict.cpp \
    ilp/ILP.cpp \
    ilp/Instance.cpp \
    ilp/Intervals.cpp \
    cli/evaluator.cpp \
    map/sizecomputer.cpp \
    conflicts/conflictgraph.cpp \
    map/zoomcomputer.cpp \
    map/trajectoryfactory.cpp \
    config.cpp \
    util/setrtree.cpp \
    heuristics/intervalgraphheuristic.cpp \
    heuristics/utils.cpp \
    checks/resultconsistencychecker.cpp \
    checks/nooverlapschecker.cpp \
    util/debugging.cpp \
    clipper.cpp

HEADERS  += ui/labelrotation.h \
    map/mapreader.h \
    map/map.h \
    map/router.h \
    ui/drawingcontroller.h \
    ui/widgets/sceneview.h \
    map/trajectory.h \
    map/projection.h \
    util/algvec.h \
    util/coordinateutil.h \
    ui/poifilterchooser.h \
    conflicts/rotatingpoi.h \
    conflicts/conflict.h \
    conflicts/rotatingpoifactory.h \
    conflicts/camera.h \
    map/trajectoryinterpolator.h \
    ui/widgets/cameraview.h \
    conflicts/trajectoryfilter.h \
    app.h \
    app.h \
    cli/clirunner.h \
    config.h \
    tests/conflicttest.h \
    heuristics/heuristic.h \
    util/setrtree.h \
    heuristics/greedyheuristic.h \
    ilp/adapter.h \
    util/clock.h \
    ilp/Conflict.h \
    ilp/ILP.h \
    ilp/Instance.h \
    ilp/Intervals.h \
    cli/evaluator.h \
    map/sizecomputer.h \
    conflicts/conflictgraph.h \
    map/zoomcomputer.h \
    map/trajectoryfactory.h \
    heuristics/intervalgraphheuristic.h \
    heuristics/utils.h \
    checks/resultconsistencychecker.h \
    checks/nooverlapschecker.h \
    util/debugging.h \
    clipper.h

FORMS    += ui/labelrotation.ui \
    ui/poifilterchooser.ui

INCLUDEPATH += "contrib/osmium/include" "/home/benjamin/programs/gurobi605/linux64/include/" $$(GUROBI_HOME)/include


!exists( "/usr/include/boost/geometry/algorithms/dispatch/disjoint.hpp" ) {
  QMAKE_CXXFLAGS += -DOLD_BOOST
}

QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS += -std=c++11 -fext-numeric-literals -g -O0

LIBS += -lexpat -lCGAL -lgmp -lmpfr -lboost_thread -lboost_system -lboost_program_options -L$$(GUROBI_HOME)/lib  -L/home/benjamin/programs/gurobi605/linux64/lib  -lgurobi_c++ -lgurobi65


#QMAKE_CFLAGS += -pg
#QMAKE_CXXFLAGS += -O0
#QMAKE_LFLAGS += -pg

unix|win32: LIBS += -lproj
unix|win32: LIBS += -llapack
unix|win32: LIBS += -larmadillo
unix|win32: LIBS += -lblas
