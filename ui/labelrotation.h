#ifndef LABELROTATION_H
#define LABELROTATION_H

#include <QMainWindow>

#include <QGraphicsView>
#include <QSplitter>

#include "ui/drawingcontroller.h"
#include "map/map.h"


typedef struct st_screenshot_info {
  const char *filename;
  int heuristic;
  long seed;
  double position;
  int k;
} screenshot_info;


namespace Ui {
class LabelRotation;
}

class LabelRotation : public QMainWindow
{
    Q_OBJECT

public:
    explicit LabelRotation(Map *map, screenshot_info *sinfo = nullptr, QWidget *parent = 0);
    ~LabelRotation();

    void set_step(double step, int total);

private Q_SLOTS:
    void on_routeButton_clicked();
    void on_poiSelectButton_clicked();
    void on_poi_selected(std::map<std::string, std::set<std::string>> filter);

    void on_playButton_clicked();

    void on_doubleSpinBox_valueChanged(double arg1);

    void on_step_changed(double step, int total, double angle);

    void on_stepSpinner_valueChanged(double arg1);

    void on_stepSpinner_editingFinished();

    void on_stepSlider_valueChanged(int value);

    void on_runHeuristicButton_clicked();

    void on_viewport_info(double angle, double zoom);

    void on_saveButton_clicked();

    void on_angleSpinBox_valueChanged(double arg1);

private:
  void do_screenshot();

    Ui::LabelRotation *ui;
    DrawingController controller;
    const Map *map;

    QGraphicsView *mapView;
    QGraphicsView *cameraView;
    QSplitter *viewSplitter;

    screenshot_info *sinfo;
};

#endif // LABELROTATION_H
