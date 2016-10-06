#ifndef PREDSPRACOVANIE_H
#define PREDSPRACOVANIE_H

#include <QMainWindow>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv2/core/mat.hpp"
#include <QGraphicsPixmapItem>

using namespace cv;

namespace Ui {
class Predspracovanie;
}

class Predspracovanie : public QMainWindow
{
    Q_OBJECT

public:
    explicit Predspracovanie(QWidget *parent = 0);
    ~Predspracovanie();
    Mat segmentation (Mat mat,int velkost_bloku);
    Mat normalisation(Mat odtlacok,double variance,double mean);
    Mat adaptive_normalisation(Mat odtlacok,double variance,double mean,int velkost_bloku,double alfa1,double alfa2);
     void PixMap2Mat(const QPixmap& img);
     QPixmap Mat2QPixmap(const Mat &matica);
     double variancia2(Mat odtlacok,double str);
     double mean_cely_obr(Mat odtlacok);
     void calculateOrientations(Mat odtlacok,int velkost_bloku);
     void UpdateView(Mat mat);
     void UpdateView0(Mat mat);
     void frekvencieLinii(Mat normaliz,Mat smer,int velkost_bloku);
     void exportujMapuSmeru();
     void gaborFilter(int velkost);
     void vyhladenieMapy(Mat mapa);
     void binarizacia(Mat img);
     void adaptive_segmentation(Mat mat,int velkost_bloku,double w1,double w2,double w3,double w4);
     double mean_nomask(Mat odtlacok);
     void thinningIteration(Mat& img, int iter);
     void thinning(Mat& img, Mat filtrovana);
     double transformation(double delta);
     void loadFingerprints();
     double fann_smery[812];
     int globBlockSize = 17;

    QGraphicsPixmapItem *on_pushButton_clicked(QString subor);

    void on_comboBox_activated(const QString &arg1);

    void on_comboBox_2_activated(const QString &arg1);

    void on_pushButton_6_clicked();

    Mat mat,mat_po_segmentacii,mat_po_normal,mat_filtrovana,kostra, maska, orientation, kostra_signs, Theta;

    QPixmap fingerprint;

private slots:

private:
    Ui::Predspracovanie *ui;
    Mat frekvencnaMat,zobrazFrekvencnu,sigma;

    QGraphicsPixmapItem *item;
    QString path;
};

#endif // PREDSPRACOVANIE_H
