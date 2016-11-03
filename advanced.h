#ifndef ADVANCED_H
#define ADVANCED_H

#include <QDialog>

#include "predspracovanie.h"
#include <QGraphicsPixmapItem>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

struct minutiae{
   int x;
   int y;
   int type;
};

namespace Ui {
class Advanced;
}

class Advanced : public QDialog
{
    Q_OBJECT

public:
    explicit Advanced(QWidget *parent = 0);
    ~Advanced();
    void PixMap2Mat(const QPixmap &img);
    QPixmap Mat2QPixmap(const Mat &matica);
    QPixmap Mat2QPixmap_RGB(const Mat &src);
    void UpdateView(Mat mat);
    void UpdateView0(Mat mat);
    bool give_global();

    Ui::Advanced *ui;
    Predspracovanie *p;
    QGraphicsPixmapItem *item;
    String network_file;
    vector <vector<int> > positions;
    vector <minutiae> detected_minutiae;
    vector <minutiae> real_minutiae;
    Mat output_cn;
    QString basic_filename;
    int mask_sum;
    String global_net;
    int global_class;
    int treshold = 18;

public slots:
    void on_preprocessButton_clicked();
    void on_extract_clicked();

private slots:

    void on_open_clicked();
    void on_comboBox_2_activated(const QString &arg1);
    void on_comboBox_activated(const QString &arg1);


    void search(Mat img);
    void draw_sign(Mat img);
    Mat crossingNum(Mat img);

    void on_comboBox_4_activated(int index);

    void on_comboBox_3_activated(int index);

    void on_learn_NN_clicked();

    void on_learn_global_clicked();

    void on_comboBox_5_activated(int index);

    Mat draw_detected(Mat img, vector <minutiae> detected);

    void on_file_automatic_clicked();

    void on_pushButton_2_clicked();

private:

};

#endif // ADVANCED_H
