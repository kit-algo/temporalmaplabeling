#include "cameraview.h"

#include <QDebug>

CameraView::CameraView(int w, int h, QWidget *parent):
    QGraphicsView(parent), w(w), h(h)
{
  this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  this->setFrameStyle(QFrame::Box | QFrame::Plain);
}

#define ASPECT_DELTA 1
void
CameraView::resizeEvent(QResizeEvent * event)
{
  std::cout << "New size: " << event->size().width() << "x" << event->size().height();

  double aspect_ratio = (1.0*this->h) / this->w;
  int wanted_height = event->size().width() * aspect_ratio;
  if (std::abs(wanted_height - event->size().height()) <= ASPECT_DELTA) {
    return;
  } else {
    this->resize(event->size().width(), wanted_height);
    std::cout << "Resizing to " << event->size().width() << "x" << wanted_height;
  }
}

void CameraView::wheelEvent(QWheelEvent *event)
{
  (void)event;
    /*
    if(event->delta() > 0)
    {
        this->scale(1.3, 1.3);
    }
    else
    {
        this->scale((1/1.3), (1/1.3));
    }
    */
}
