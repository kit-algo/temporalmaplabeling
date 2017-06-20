#ifndef POIFILTERCHOOSER_H
#define POIFILTERCHOOSER_H

#include <QDialog>
#include <QStandardItemModel>

#include "map/map.h"

namespace Ui {
class POIFilterChooser;
}

class POIFilterChooser : public QDialog
{
    Q_OBJECT

public:
    explicit POIFilterChooser(const std::set<POI*> pois, QWidget *parent = 0);
    ~POIFilterChooser();

Q_SIGNALS:
    void onClose(std::map<std::string, std::set<std::string>> selection);

private Q_SLOTS:
    void onTreeClicked(QModelIndex index);

    void on_buttonBox_accepted();

    void on_buttonBox_rejected();


private:
    Ui::POIFilterChooser *ui;
    const std::set<POI*> pois;
    QStandardItemModel treeModel;

    void buildTree();
    std::map<std::string, std::set<std::string>> collectFilters();
};

#endif // POIFILTERCHOOSER_H
