#ifndef DRAWINGCONTROLLER_H
#define DRAWINGCONTROLLER_H

#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QTimer>

#include "map/map.h"
#include "map/trajectory.h"
#include "conflicts/camera.h"
#include "heuristics/heuristic.h"
#include "map/trajectoryfactory.h"

class DrawingController : public QObject
{
    Q_OBJECT

public:
    DrawingController(Map *map);
    void attach(QGraphicsView *view, QGraphicsView *cameraView);

    void new_route(int seed);

    void set_poi_filter(std::map<std::string, std::set<std::string>> filter);

    void start_play();

    void force_angle(double angle);
    void set_play_point(double point);

    void compute_heuristic(int heuristic_tag, int k = -1);

    void save_camera(const char *filename);

Q_SIGNALS:
    void step_changed(double step, int total, double angle);
    void viewport_info(double angle, double zoom);

private Q_SLOTS:
    void update_play_point();

private:
    QGraphicsScene scene;
    QGraphicsView *view;

    QGraphicsScene cameraScene;
    QGraphicsView *cameraView;
    double camera_scale_factor;

    std::vector<QGraphicsItem *> drawn_poi;
    std::vector<QGraphicsItem *> drawn_trajectory;
    std::vector<QGraphicsItem *> viewportItems;

    void redraw();
    QGraphicsItemGroup * redrawPOI();

    QTimer *play_timer;
    double play_point;
    double expected_point;

    void showViewportAt(double dist);
    void updateCameraTransform(RotatingPOI &viewport, CarPosition carpos);

    QGraphicsItemGroup *draw_ways();
    QGraphicsItem *draw_waypoints(std::vector<Position> waypoints, double speed);
    QGraphicsItem *draw_route(std::vector<Position> waypoints);
    QGraphicsItemGroup *draw_crossings();

    void draw_trajectory();
    QGraphicsItem *draw_trajectory_straight(StraightTrajectoryItem *i);
    QGraphicsItem *draw_trajectory_circle(CircleTrajectoryItem *i);

    Map *map;

    Trajectory *trajectory;
    Camera *camera;
    double forced_angle;
    SelectionIntervals *selected;

    void save_camera_svg(const char *filename);
};

#endif // DRAWINGCONTROLLER_H
