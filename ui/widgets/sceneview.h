#ifndef SCENEVIEW_H
#define SCENEVIEW_H

#include <QGraphicsView>
#include <QWidget>
#include <QWheelEvent>

class SceneView : public QGraphicsView
{
public:
    SceneView(QWidget * parent = 0);

protected:
    void wheelEvent(QWheelEvent *event);
};

#endif // SCENEVIEW_H
