#include "sceneview.h"

#include <QGraphicsView>

SceneView::SceneView(QWidget *parent):
    QGraphicsView(parent)
{
    this->setDragMode(QGraphicsView::ScrollHandDrag);
    this->setFrameStyle(QFrame::Box | QFrame::Plain);
    //this->setRenderHints(0);
}

void SceneView::wheelEvent(QWheelEvent *event)
{
    if(event->delta() > 0)
    {
        this->scale(1.3, 1.3);
    }
    else
    {
        this->scale((1/1.3), (1/1.3));
    }
}
