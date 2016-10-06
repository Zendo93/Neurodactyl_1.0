#ifndef AUTOMATIC_H
#define AUTOMATIC_H

#include <QDialog>

#include "predspracovanie.h"
#include "advanced.h"
#include <QGraphicsPixmapItem>

using namespace cv;

namespace Ui {
class Automatic;
}

class Automatic : public QDialog
{
    Q_OBJECT

public:
    explicit Automatic(QWidget *parent = 0);
    ~Automatic();
    void UpdateView(Mat mat);
    void compare(QString global);

private slots:
    void on_file_automatic_clicked();

private:
    Ui::Automatic *ui;
    Advanced *adv;
    QGraphicsPixmapItem *item;

};

#endif // AUTOMATIC_H
