#ifndef CAMERAVIEW_H
#define CAMERAVIEW_H

#include <QWidget>
#include <QGraphicsView>
#include <QWheelEvent>

#include <iostream>

class CameraView : public QGraphicsView
{
public:
    CameraView(int w, int h, QWidget * parent = 0);

protected:
    void wheelEvent(QWheelEvent *event);
    void resizeEvent(QResizeEvent * event);

    int w;
    int h;
};

#endif // CAMERAVIEW_H
