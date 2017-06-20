#include "ui/labelrotation.h"
#include "ui_labelrotation.h"

#include "ui/poifilterchooser.h"

#include "ui/widgets/cameraview.h"
#include "ui/widgets/sceneview.h"

#include "config.h"

#include <QSplitter>
#include <QDebug>

void
LabelRotation::do_screenshot()
{
  this->controller.new_route(this->sinfo->seed);
  this->controller.compute_heuristic(this->sinfo->heuristic, this->sinfo->k);
  // set size
  /*
  int total_size = 0;
  for (int size : this->viewSplitter->sizes()) {
    std::cout << "Got a size: " << size << "\n";
    total_size += size;
  }
  QList<int> newSizes;
  assert(total_size > CAMERA_WIDTH);

  newSizes += (total_size - CAMERA_WIDTH);
  newSizes += CAMERA_WIDTH;
  qDebug() << "Setting sizes to: " << newSizes;
  this->viewSplitter->setSizes(newSizes);
  */

  this->cameraView->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
  this->cameraView->resize(CAMERA_WIDTH, CAMERA_HEIGHT);

  this->controller.set_play_point(this->sinfo->position);
  this->controller.save_camera(this->sinfo->filename);
}

LabelRotation::LabelRotation(Map *map, screenshot_info *sinfo, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LabelRotation),
    controller(map),
    map(map),
    sinfo(sinfo)
{
    ui->setupUi(this);

    this->setWindowFlags(Qt::Window);
    this->setWindowState(Qt::WindowMinimized);

    this->viewSplitter = new QSplitter(ui->scenesFrame);
    this->mapView = new SceneView();
    this->cameraView = new CameraView(CAMERA_WIDTH, CAMERA_HEIGHT);
    this->cameraView->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    this->viewSplitter->addWidget(this->mapView);


    QFrame *cameraFrame = new QFrame();
    QVBoxLayout *cameraFrameLayout = new QVBoxLayout();
    //cameraFrameLayout->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    cameraFrame->setLayout(cameraFrameLayout);
    //cameraFrame->setFrameStyle(QFrame::Box | QFrame::Plain);
    cameraFrameLayout->addWidget(this->cameraView);


    this->viewSplitter->addWidget(cameraFrame);
    //this->viewSplitter->layout()->setAlignment(Qt::AlignRight | Qt::AlignVCenter);


    this->viewSplitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    //layout->addWidget(this->viewSplitter, 0, 0);
    //ui->scenesFrame->setLayout(layout);
    ui->scenesFrame->layout()->addWidget(this->viewSplitter);

    this->controller.attach(this->mapView, this->cameraView);
    connect(&(this->controller), SIGNAL(step_changed(double,int,double)), this, SLOT(on_step_changed(double,int,double)));
    connect(&(this->controller), SIGNAL(viewport_info(double, double)), this, SLOT(on_viewport_info(double, double)));

    /* Initialize available heuristics */
    ui->heuristicComboBox->insertItem(0, "Greedy AM1", QVariant(HEU_GREEDY_AM1_TAG));
    ui->heuristicComboBox->insertItem(1, "Greedy AM2", QVariant(HEU_GREEDY_AM2_TAG));
    ui->heuristicComboBox->insertItem(2, "Greedy AM3", QVariant(HEU_GREEDY_AM3_TAG));
    ui->heuristicComboBox->insertItem(3, "ILP AM1", QVariant(HEU_ILP_AM1_TAG));
    ui->heuristicComboBox->insertItem(4, "ILP AM2", QVariant(HEU_ILP_AM2_TAG));
    ui->heuristicComboBox->insertItem(5, "ILP AM3", QVariant(HEU_ILP_AM3_TAG));

    this->showMaximized();

    if (this->sinfo != nullptr) {
      std::cout << "Creating screenshot…\n";
      this->do_screenshot();
      exit(0);
    }
}

LabelRotation::~LabelRotation()
{
    delete ui;
}

void LabelRotation::set_step(double step, int total)
{
    this->ui->totalStepsLabel->setText(QString::number(total));
    this->ui->stepSpinner->setValue(step);
    this->ui->stepSpinner->setMaximum(total - 1);
    if (step == std::floor(step)) {
      this->ui->stepSlider->setRange(0, total - 1);
      this->ui->stepSlider->setValue((int)step);
    }
}

void LabelRotation::on_viewport_info(double angle, double zoom) {
    this->ui->label_cur_zoom->setText(QString::number(zoom));
    this->ui->label_cur_angle->setText(QString::number(angle));
}

void LabelRotation::on_routeButton_clicked()
{
    this->controller.new_route(this->ui->seedSpinBox->value());
}

void LabelRotation::on_poiSelectButton_clicked()
{
    POIFilterChooser *pfc = new POIFilterChooser(this->map->getPOI(), this);
    pfc->setAttribute(Qt::WA_DeleteOnClose);
    connect(pfc, SIGNAL(onClose(std::map<std::string,std::set<std::string> >)), this, SLOT(on_poi_selected(std::map<std::string,std::set<std::string> >)));
    //connect(pfc, SIGNAL(onClose(std::map<std::string,std::set<std::string> > selection)), this, SLOT(on_poi_selected(std::map<std::string,std::set<std::string> > filter)));
    pfc->show();
}

void LabelRotation::on_poi_selected(std::map<std::string, std::set<std::string> > filter)
{
    this->controller.set_poi_filter(filter);
}

void LabelRotation::on_playButton_clicked()
{
    this->controller.start_play();
}

void LabelRotation::on_doubleSpinBox_valueChanged(double angle)
{
  // Not used anymore
  (void) angle;
}

void LabelRotation::on_step_changed(double step, int total, double angle)
{
  (void)angle;
  this->set_step(step, total);
}

void LabelRotation::on_stepSpinner_valueChanged(double arg1)
{
  std::cout << "Setting PP to " << arg1 << "\n";
  this->controller.set_play_point(arg1);
}

void LabelRotation::on_stepSpinner_editingFinished()
{

}

void LabelRotation::on_stepSlider_valueChanged(int value)
{
    this->controller.set_play_point(value);
}

void LabelRotation::on_runHeuristicButton_clicked()
{
    int index = this->ui->heuristicComboBox->currentIndex();
    int tag = this->ui->heuristicComboBox->itemData(index).toInt();
    this->controller.compute_heuristic(tag);
}

void LabelRotation::on_saveButton_clicked()
{
    this->controller.save_camera("/tmp/screenshot.svg");
}

void LabelRotation::on_angleSpinBox_valueChanged(double arg1)
{
    this->controller.force_angle(arg1);
}
