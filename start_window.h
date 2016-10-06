#ifndef START_WINDOW_H
#define START_WINDOW_H

#include <QDialog>

#include "advanced.h"
#include "automatic.h"

namespace Ui {
class Start_window;
}

class Start_window : public QDialog
{
    Q_OBJECT

public:
    explicit Start_window(QWidget *parent = 0);
    ~Start_window();

private slots:
    void on_advanced_clicked();

    void on_identification_clicked();

    void on_pushButton_clicked();

private:
    Ui::Start_window *ui;
    Advanced *advanced;
    Automatic *automatic;
};

#endif // START_WINDOW_H
