#ifndef APP_H
#define APP_H

#include <QApplication>
#include <QEvent>

class App : public QApplication
{
public:
    App(int &argc, char *argv[]);
    bool notify(QObject * receiver, QEvent * e);
};

#endif // APP_H
