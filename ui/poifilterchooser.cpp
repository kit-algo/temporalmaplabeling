#include "poifilterchooser.h"
#include "ui_poifilterchooser.h"

#include "map/map.h"
#include <QStandardItem>
#include <QString>

POIFilterChooser::POIFilterChooser(const std::set<POI *> pois, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::POIFilterChooser),
    pois(pois)
{
    ui->setupUi(this);
    this->buildTree();
}

POIFilterChooser::~POIFilterChooser()
{
    delete ui;
}

void POIFilterChooser::onTreeClicked(QModelIndex index)
{
    QStandardItem *item = this->treeModel.itemFromIndex(index);

    for (int r=0; r<item->rowCount(); r++)
    {
        QModelIndex childIndex = this->treeModel.index(r,0,index);
        QStandardItem *child = this->treeModel.itemFromIndex(childIndex);
        child->setCheckState(item->checkState());
    }
}

void POIFilterChooser::buildTree()
{
    for (auto key : POI::CLASS_KEYS) {
        QStandardItem *key_root = new QStandardItem(QString::fromStdString(key));
        key_root->setCheckable(true);
        this->treeModel.appendRow(key_root);

        std::set<std::string> seen;
        for (auto poi : this->pois) {
            std::string value = poi->getClass(key);
            if (value == "")
                continue;

            if (seen.find(value) != seen.end())
                continue;

            QStandardItem *val_item = new QStandardItem(QString::fromStdString(value));
            val_item->setCheckable(true);
            val_item->setData(QString::fromStdString(key));
            key_root->appendRow(val_item);
            seen.insert(value);
        }
    }

    this->ui->treeView->setModel(&(this->treeModel));

    connect(this->ui->treeView, SIGNAL(clicked(QModelIndex)), this, SLOT(onTreeClicked(QModelIndex)));
}

std::map<std::string, std::set<std::string>> POIFilterChooser::collectFilters() {
    std::map<std::string, std::set<std::string>> result;

    std::vector<QStandardItem *> roots;
    roots.push_back(this->treeModel.invisibleRootItem());

    while (roots.size() > 0) {
        QStandardItem *item = roots.back();
        roots.pop_back();
        QVariant data = item->data();
        if (data.isValid() && item->checkState() == Qt::Checked) {
            std::string key = data.toString().toStdString();
            std::string value = item->text().toStdString();

            if (result.find(key) != result.end()) {
                result[key].insert(value);
            } else {
                result[key] = { value };
            }
        }

        if (item->hasChildren()) {
            QModelIndex index = this->treeModel.indexFromItem(item);
            for (int r=0; r<item->rowCount(); r++)
            {
                QModelIndex childIndex = this->treeModel.index(r,0,index);
                QStandardItem *child = this->treeModel.itemFromIndex(childIndex);
                roots.push_back(child);
            }
        }
    }

    return result;
}

void POIFilterChooser::on_buttonBox_accepted()
{
    Q_EMIT onClose(this->collectFilters());
    this->close();
}

void POIFilterChooser::on_buttonBox_rejected()
{
    this->close();
}
