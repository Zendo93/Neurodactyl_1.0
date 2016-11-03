#include <QFileDialog>
#include "Graphics_view_zoom.h"
#include <opencv/cv.h>
#include <opencv2\core\mat.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2/highgui/highgui.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QRgb>
#include <QVector>
#include <fstream>
#include <iostream>
#include <QMessageBox>
#include <set>
#include "persistence1d.h"
#include <QDebug>
#include <QElapsedTimer>
#define PI 3.14159265
#include "predspracovanie.h"
#include "ui_predspracovanie.h"
#define PI_2 CV_PI/2
#include <opencv2\gpu\gpu.hpp>

using namespace cv;
using namespace std;
using namespace p1d;

Predspracovanie::Predspracovanie(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Predspracovanie)
{
    ui->setupUi(this);
}

Predspracovanie::~Predspracovanie()
{
    delete ui;
}
void Predspracovanie::PixMap2Mat(const QPixmap &img){                  //funkcia nutná na konverziu 8bit obrazu z povodnej QPixmap
                                                                 //ktoru vyuziva QT Creator, vystupom je Maticu obrazu na pracu v OpenCV
    QImage image = img.toImage();
    QColor rgb;
    this->mat= Mat(image.height(),image.width(),CV_8UC1);
    for(int i=0;i<this->mat.rows;i++){
        for(int j=0;j<this->mat.cols;j++){
            rgb=image.pixel(j,i);
            this->mat.at<uchar>(i,j)=abs(rgb.black()-255);
        }
    }
}

QPixmap Predspracovanie::Mat2QPixmap(const Mat &matica){   //Opacna konverzia OpenCV matice na maticu umoznujucu zobrazenie v Grafickom prostredi
    static QVector<QRgb> sColorTable;
    if(sColorTable.isEmpty()){
        for(int i=0;i<256;i++){
            sColorTable.push_back(qRgb(i,i,i));
        }
    }

   QImage img(matica.data, matica.cols, matica.rows, matica.step, QImage::Format_Indexed8);
   img.setColorTable(sColorTable);
   return QPixmap::fromImage(img);

}

double variancia(Mat odtlacok){ //vypocita varianciu v obraze bez vypocitanej segmentacnej mapy
    Scalar str,intensity;
    double stary,variancia=0,pomocna;
    str = cv::mean(odtlacok);
    for(int i=0; i<odtlacok.rows;i++){
        uchar* rowi = odtlacok.ptr(i);
        for(int j=0; j<odtlacok.cols; j++){
                intensity=rowi[j];
                pomocna=(intensity[0]-str[0]);
                stary=pow(pomocna,2);
                variancia=variancia+stary;
  }
}
    variancia=variancia/(odtlacok.rows*odtlacok.cols);
    return variancia;
}

void vykreslenieCiary(Mat mapa, int velkost_bloku, int i, int j, double uhol){ //Vykresluje smerovu mapu odtlacku
    double x1, y1, x2, y2;
    x1 = velkost_bloku / 2.0 + (i*velkost_bloku);
    y1 = velkost_bloku / 2.0 + (j*velkost_bloku);
    x2 = velkost_bloku + (i*velkost_bloku);
    y2 = velkost_bloku / 2.0 + (j*velkost_bloku);
    Point bod_vypocitany(((x2 - x1)*cos(uhol) - (y2 - y1)*sin(uhol)) + velkost_bloku / 2.0 + (i*velkost_bloku), ((x2 - x1)*sin(uhol) + (y2 - y1)*cos(uhol)) + velkost_bloku / 2.0 + (j*velkost_bloku));
    Point bod_staticky(x1, y1);
    line(mapa, bod_staticky, bod_vypocitany, Scalar(255, 255, 255), 2, 4, 0); //farba,hrubka,typ,shift
}

double Predspracovanie::mean_cely_obr(Mat odtlacok){ // vypocet strednej hodnoty pre celý vstupny obraz
    double stary=0,mean,d;
    for(int i=0; i<odtlacok.rows;i++){
        for(int j=0; j<odtlacok.cols; j++){
            d=this->maska.at<uchar>(i,j);
            if(d!=0){                           //dba na vypocitanu masku
                d=odtlacok.at<uchar>(i,j);
                stary=stary+d;
            }
  }
}
    mean=stary/(odtlacok.rows*odtlacok.cols); //vysledok strednej hodnoty
    return mean;
}
double Predspracovanie::mean_nomask(Mat odtlacok){   //vypocet strednej hodnoty obrazu bez vypocitanej masky
    double stary=0,mean,d;
    for(int i=0; i<odtlacok.rows;i++){
        for(int j=0; j<odtlacok.cols; j++){
                d=odtlacok.at<uchar>(i,j);
                stary=stary+d;

  }
}
    mean=stary/(odtlacok.rows*odtlacok.cols); //vysledok segmentacie
    return mean;
}

void Predspracovanie::adaptive_segmentation(Mat mat,int velkost_bloku,double w1,double w2,double w3,double w4){ //AdaptĂ­vna segmentĂˇcia vracia masku obrazu a segmentovany obraz
    Mat vysledok,submatica,X(1,4,CV_64F),W(4,1,CV_64F);
    double var,mean,dom_score,seg_score=0;
    this->maska = Mat(mat.size(),CV_8UC1,Scalar::all(255));//naplnime masku hodnotami bielej
    mat.copyTo(vysledok);
    int riadky=floor(mat.rows/velkost_bloku);
    int stlpce=floor(mat.cols/velkost_bloku);
    double mi;
    mi = pow(10,-4)/(1+(pow(10,6)/pow(10,4)));
    W.at<double>(0,0) = w1;
    W.at<double>(1,0) = w2;
    W.at<double>(2,0) = w3;
    W.at<double>(3,0) = w4;

    for(int x=0;x<riadky;x++){
        for(int y=0;y<stlpce;y++){
           submatica=vysledok(Range(x*velkost_bloku,x*velkost_bloku+velkost_bloku-1),Range(y*velkost_bloku,y*velkost_bloku+velkost_bloku-1)); //vyberie informaciu na bloku i,j
           mean = mean_nomask(submatica);
           var = variancia(submatica);
           dom_score = ((this->Theta.at<double>(x,y)+CV_PI/2)*180/CV_PI)/(pow(velkost_bloku,2)); //dominantne skore potrebne vo vektore X
           X.at<double>(0,0) = dom_score;
           X.at<double>(0,1) = mean;
           X.at<double>(0,2) = var;
           X.at<double>(0,3) = 1;
           for(int j=0;j<W.rows;j++){
               seg_score += X.at<double>(0,j) * W.at<double>(j,0); //segmentacne skore
           }
           if(seg_score <= 0){
               for(int i=0;i<velkost_bloku;i++){
                   for(int j = 0;j<velkost_bloku;j++){
                       vysledok.at<uchar>(x*velkost_bloku+i,y*velkost_bloku+j) = 0;
                       this->maska.at<uchar>(x*velkost_bloku+i,y*velkost_bloku+j) = 0;
                   }
               }
           }

           seg_score = 0;

        }
    }
    UpdateView(vysledok); //V grafickom prostredĂ­ sa zobrazĂ­ segmentovanĂ˝ obraz
}

Mat Predspracovanie::segmentation (Mat mat,int velkost_bloku){ // funkcia vykonavajuca blokovu segmentaciu
    double var_submat;
    Mat submatica;
    Mat odtlacok;
    mat.copyTo(odtlacok);
    Mat mask(mat.size(),CV_8UC1,Scalar::all(255));
    Scalar sedost_submat;
    Scalar sedost_cela= mean(odtlacok);
    double minVal, maxVal,minVal1, maxVal1;
    minMaxLoc(odtlacok, &minVal, &maxVal);
    double var_cely = variancia(odtlacok);
    int riadky=floor(odtlacok.rows/velkost_bloku);
    int stlpce=floor(odtlacok.cols/velkost_bloku);
        for(int x=0;x<riadky;x++){
            for(int y=0;y<=stlpce;y++){
                if(y<stlpce){
                submatica=odtlacok(Range(x*velkost_bloku,x*velkost_bloku+velkost_bloku-1),Range(y*velkost_bloku,y*velkost_bloku+velkost_bloku-1)); //toto je blok v ktorom sa porovnava var a mean
                minMaxLoc(submatica, &minVal1, &maxVal1);
                var_submat=variancia(submatica);
                sedost_submat=mean(submatica);
                if((var_submat<(var_cely))  &&  (sedost_submat[0]>=(sedost_cela[0]+((maxVal-minVal1)/2)))){
                    for(int i=0; i<velkost_bloku;i++){
                        uchar* rowi1 = odtlacok.ptr(i+x*velkost_bloku);
                        uchar* rowi2 = mask.ptr(i+x*velkost_bloku);
                        for(int j=0; j<velkost_bloku; j++){
                            rowi1[j+y*velkost_bloku]=0;
                            rowi2[j+y*velkost_bloku]=0;
                            }
                        }
                    }
                }
            }
        }
        mask.copyTo(this->maska);//zapis do masky
    return odtlacok;//vracia odtlacok
}

double Predspracovanie::variancia2(Mat odtlacok,double str){// vypocita varianciu v segmentovanom obraze
    Scalar intensity;
    double stary,variancia=0,pomocna,d;
    for(int i=0; i<odtlacok.rows;i++){
        uchar* rowi = odtlacok.ptr(i);
        for(int j=0; j<odtlacok.cols; j++){
            d=this->maska.at<uchar>(i,j);
                if(d!=0){                         //nedba na okolie odtlacku, rata iba v papilarnom terene
                intensity=rowi[j];
                pomocna=(intensity[0]-str);
                stary=pow(pomocna,2);
                variancia=variancia+stary;
            }
  }
}
    variancia=variancia/(odtlacok.rows*odtlacok.cols);
    return variancia;
}

Mat Predspracovanie::normalisation(Mat odtlacok,double variance,double mean){ //globalna normalizacia odtlacku
    double mean_cely = mean_cely_obr(odtlacok);
    double variancia_cela =variancia2(odtlacok,mean_cely);
    Mat odtlacok1;
    odtlacok.copyTo(odtlacok1);
    double d;
    mean=0-mean;
    for(int i=0;i<odtlacok1.rows;i++){
        for(int j=0;j<odtlacok1.cols;j++){
            d=this->maska.at<uchar>(i,j);
            if(d!=0){
                d=odtlacok1.at<uchar>(i,j);
                if(d>mean_cely){ //ak je svetly
                   d=odtlacok1.at<uchar>(i,j);
                  odtlacok1.at<uchar>(i,j)=abs(mean+(sqrt(variance*(pow(d-mean_cely,2))/variancia_cela))-255);
                }                                       //abs hodnota je o5 potrebna koli inverznemu mapovaniu hodnot QPixmap
                else{
                   d=odtlacok1.at<uchar>(i,j);
                  odtlacok1.at<uchar>(i,j)=abs(mean-(sqrt(variance*(pow(d-mean_cely,2))/variancia_cela))-255);
                }
            }
        }
    }
    return odtlacok1;//vystupom je normalizovany odtlacok
}

Mat Predspracovanie::adaptive_normalisation(Mat odtlacok,double variance,double mean1,int velkost_bloku,double alfa1,double alfa2){//funkcia vykonavajuca adaptivnu normalizaciu
    double new_var, new_mean,alpha1=alfa1,alpha2=alfa2;
    Mat odtlacok1,submatica;
    odtlacok.copyTo(odtlacok1);
    int rows = odtlacok1.rows/velkost_bloku;
    int cols = odtlacok1.cols/velkost_bloku;
    double d;
    double mean_cely = mean_cely_obr(odtlacok1);
    double variancia_cela = variancia(odtlacok1);
    for(int x=0;x<rows;x++){
        for(int y=0;y<cols;y++){
            submatica=odtlacok1(Range(x*velkost_bloku,x*velkost_bloku+velkost_bloku),Range(y*velkost_bloku,y*velkost_bloku+velkost_bloku)); //vyberam blok
            new_mean = mean1;
            new_var = variance;
            double mean_bloku = mean_cely_obr(submatica); //vypocita sa mean v bloku

            double variancia_bloku = variancia(submatica); //vypocita sa variancia v bloku
            new_mean=new_mean + alpha1*(mean_bloku-new_mean); //tu sa vykonava ta adaptivnost
            new_var = new_var + alpha2*(variancia_bloku-new_var);
            for(int i=0;i<submatica.rows;i++){
                for(int j=0;j<submatica.cols;j++){
            d=this->maska.at<uchar>(i+x*velkost_bloku,j+y*velkost_bloku);
            if(d!=0){                                                       ///ak maska v bode neobsahuje ciernu farbu
                d=odtlacok1.at<uchar>(i+x*velkost_bloku,j+y*velkost_bloku);

                if(d>mean_cely){ //ak je svetly
                   d=odtlacok1.at<uchar>(i+x*velkost_bloku,j+y*velkost_bloku);
                  odtlacok1.at<uchar>(i+x*velkost_bloku,j+y*velkost_bloku)=abs(new_mean+(sqrt((new_var*(pow(d-mean_cely,2)))/variancia_cela))-255);
                }                                                                   //absolutna hodnota je potrebna lebo Qpixmap mapuje hodnoty inverzne
                else{
                   d=odtlacok1.at<uchar>(i+x*velkost_bloku,j+y*velkost_bloku);
                  odtlacok1.at<uchar>(i+x*velkost_bloku,j+y*velkost_bloku)=abs(new_mean-(sqrt((new_var*(pow(d-mean_cely,2)))/variancia_cela))-255);
                }

            }
                }
            }
        }
    }
    binarizacia(odtlacok1); //aplikujem binarizáciu
    return odtlacok1;//vraciam normalizovaný odtlacok
}

void Predspracovanie::calculateOrientations(Mat odtlacok,int velkost_bloku)//funkcia pocita odhadovanú smerovu mapu
{
    int poz =0;
    Mat gradientX,gradientY;
    Mat odtlacok1;
    odtlacok.copyTo(odtlacok1);
    double Vx,Vy;
    int riadky = odtlacok1.rows/velkost_bloku;
    int stlpce = odtlacok1.cols/velkost_bloku;

    /// Gradient X
      Sobel( odtlacok1, gradientX, CV_64FC1, 1, 0, 3, 1, 0, BORDER_DEFAULT );

      /// Gradient Y
      Sobel( odtlacok1, gradientY, CV_64FC1, 0, 1, 3, 1, 0, BORDER_DEFAULT );

    // smerova mapa a Matica Theta s ulozenymi smermi
    this->orientation= Mat(odtlacok1.rows, odtlacok1.cols, CV_8UC1);//vykreslenie smerovej
    this->Theta = Mat(riadky, stlpce, CV_64F);//matica uhlov


    double xena, yoga;
    double vysledok;
    //Vypocet smerov a zapis do Theta

    for(int x = riadky; x--;)
    {
        for(int y = stlpce; y--;)
        {
            Vx=-0;Vy=0;
            for(int i=x*velkost_bloku;i<x*velkost_bloku+velkost_bloku;i++)
            {
                for(int j=y*velkost_bloku;j<y*velkost_bloku+velkost_bloku;j++)
                {
                   xena = (double)gradientX.at<double>(i,j);
                   yoga = (double)gradientY.at<double>(i,j);

                   Vx +=(2*(xena*yoga));  //Vypocet G_sy
                   Vy +=(pow(xena,2)-pow(yoga, 2)); //vypocet G_sx
                }
            }

            this->Theta.at<double>(x,y) = 0.5*atan2(Vx, Vy); //Zapis uhlu v radianoch do theta
        }
    }
    vyhladenieMapy(this->Theta);
    //Vykreslovanie smerovej mapy
    for (int i=0;i<riadky; i++)
    {
        for(int j=0;j<stlpce;j++)
        {
            vysledok = Theta.at<double>(i, j);
            if(vysledok > (PI_2))
            {
                vysledok -= (PI_2);
            }
            else
            {
                vysledok += (PI_2);
            }
            this->fann_smery[poz] = Theta.at<double>(i, j);
            vykreslenieCiary(orientation, velkost_bloku, j, i, vysledok);
            poz++;
        }
    }
}

double Predspracovanie::transformation(double delta) {
    if(qAbs(delta) < PI_2)
        ;
    else if(delta <= -PI_2)
        delta = PI + delta;
    else
        delta = PI - delta;

    return delta;
}


void Predspracovanie::vyhladenieMapy(Mat mapa){ //Vyhladzovanie smerovej mapy aby bola spojita
    Mat cos_Zlozka = Mat(mapa.rows, mapa.cols, CV_64F);
    Mat sin_Zlozka = Mat(mapa.rows, mapa.cols, CV_64F);
    Mat cos_filtr, sin_filtr;
    int pocet_riadkov = mapa.rows;
    int pocet_stlpcov = mapa.cols;
    for (int i = pocet_riadkov; i--;)
    {
        for (int j = pocet_stlpcov; j--;)
        {
            cos_Zlozka.at<double>(i,j) = cos(2 * mapa.at<double>(i,j));
            sin_Zlozka.at<double>(i,j) = sin(2 * mapa.at<double>(i,j));
        }
    }
    GaussianBlur(cos_Zlozka, cos_filtr, Size(globBlockSize, globBlockSize), 1,1);
    GaussianBlur(sin_Zlozka, sin_filtr, Size(globBlockSize, globBlockSize), 1, 1);
    for (int i = 0; i < cos_filtr.rows; i++)
    {
        for (int j =0; j < cos_filtr.cols; j++)
        {
            this->Theta.at<double>(i,j) =  0.5*(atan2(sin_filtr.at<double>(i, j), cos_filtr.at<double>(i, j)));
        }
    }
    //GaussianBlur( this->Theta,  this->Theta, Size(5, 5), 0.5, 0.5);
}

void Predspracovanie::binarizacia(Mat img){ //globalna binarizacia
    int i, j;
    int pocet_riadkov = img.rows;
    int pocet_stlpcov = img.cols;
    double globalny_stupen = mean_cely_obr(img);


    for (i = pocet_riadkov;  i--;){
        for (j = pocet_stlpcov; j--;){
            if (img.at<uchar>(i, j)<globalny_stupen){ //ak je menej ako glob stupeen hodnota je cierna
                img.at<uchar>(i, j) = 0;
            }
            else{
                img.at<uchar>(i, j) = 255; //inak biela
            }
        }
    }
}

void Predspracovanie::frekvencieLinii(Mat normaliz,Mat smer,int velkost_bloku){ //Odhad frekvencnej mapy
    int riadky = this->orientation.rows/velkost_bloku;
    int stlpce = this->orientation.cols/velkost_bloku;
    double uhol;
    Point2i stred;
    this->frekvencnaMat = Mat(orientation.rows,orientation.cols,CV_64F);
    this->sigma= Mat(orientation.rows,orientation.cols,CV_64F);
    this->zobrazFrekvencnu = Mat(orientation.rows,orientation.cols,CV_8UC1);
    Mat O_submat(255,750,CV_8UC1,0.0),M,crop,zrotovana; //pouziva sa iba ak chceme vykreslit sinusovy priebeh
    for(int x=0;x<riadky;x++){
        for(int y=0;y<stlpce;y++){
            stred = Point(y*velkost_bloku+velkost_bloku/2, x*velkost_bloku+velkost_bloku/2); //zistime stred bloku
            uhol = (smer.at<double>(x,y)/*+PI/2*/)*180/PI; //prevedieme uhol zo smerovej mapy do stupnov

            RotatedRect rRect(stred, Size(3*velkost_bloku,2*velkost_bloku), uhol); //vytvory sa rotovany obdlznik s vypocitanym stredom

            M = getRotationMatrix2D(rRect.center, uhol, 1.0); //obratenie aby nebol rotovany
                    // prerotuje obraz na 0 stupnov
                    warpAffine(normaliz, zrotovana, M, normaliz.size(), INTER_CUBIC);
                    // vystrihne iba ROI
                    getRectSubPix(zrotovana, rRect.size, rRect.center, crop);

                                vector<float> xSignature; //vektor Xsignatury

                                for (int k = 0; k < crop.cols; k++)
                                {
                                    int sum = 0;

                                    for (int d = 0; d < crop.rows; d++)
                                    {

                                        sum = sum + (int)crop.at<uchar>(d, k);

                                    }


                                    xSignature.push_back(sum/velkost_bloku); //vlozim hodnotu

                                }

                                vector<float> xSignature2; //xSignatura pre vypocet sigmy v Gaborovom filtri
                                for(int bla=0;bla<xSignature.size();bla++){

                                     xSignature2.push_back(abs(xSignature[bla]-255));
                                     }




                               Persistence1D p;                     //Vyhladanie lokalnych maxim v xSignature
                               p.RunPersistence(xSignature2);
                               vector< TPairedExtrema > Extrema;
                               p.GetPairedExtrema(Extrema,10);
                               vector<double> maxima;
                               vector<double> minima;
                               //ZISTIM SI MAXIMA
                               for(vector< TPairedExtrema >::iterator it = Extrema.begin(); it != Extrema.end(); it++)
                                  {
                                     maxima.push_back((*it).MaxIndex);
                                     minima.push_back((*it).MinIndex);

                                  }
                               sort(maxima.begin(),maxima.end());//SORT OD NAJNIZSIEHO PO NAJVYSSI

                               //AZ TU PREBIEHA VYPOCET
                               double sum = 0;
                               for(int i = 0; i<maxima.size();i++){//priemerny pocet pixlov medzi dvoma maximami v xSignature
                                    if(i != (maxima.size()-1)){
                                        sum += maxima[i+1] - maxima[i] - 1;
                                    }
                               }
                               double vysledok = sum/maxima.size(); //zmen na double
                               if(vysledok<0){
                                   vysledok = 0;
                               }
                                sum = 0;
                                sort(minima.begin(),minima.end()); //priemerny pocet pixlov medzi dvoma minimami v xSignature
                               for(int i = 0; i<minima.size();i++){
                                    if(i != (minima.size()-1)){
                                        sum += minima[i+1] - minima[i] - 1;
                                    }
                               }

                               double vysledok_min = sum/minima.size(); //zmen na double
                               if(vysledok_min<0){
                                   vysledok_min = 0;
                               }
                               for(int i = 0; i<velkost_bloku;i++){
                                   for(int j = 0; j<velkost_bloku;j++){
                                      this->sigma.at<double>(x*velkost_bloku+i,y*velkost_bloku+j) = vysledok_min; // hodnota urcena pre sigma v Gabore
                                      this->frekvencnaMat.at<double>(x*velkost_bloku+i,y*velkost_bloku+j) = 2*vysledok; //frekvencncia
                                     // this->zobrazFrekvencnu.at<uchar>(x*velkost_bloku+i,y*velkost_bloku+j) = (10*vysledok);//toto je pre zobrazenie v GUI
                                   }
                               }

        }
    }


}

void Predspracovanie::gaborFilter(int velkost){ //funkcia pre aplikaciu gaborovho filtera
    Mat kernel; //maska gabora
    this->mat_filtrovana = Mat(this->mat_po_normal.rows,this->mat_po_normal.cols,CV_8U,Scalar::all(0));
    double  sum = 0; //sigma nebola pouzivana nakolko sme si ju vedeli odvodit
    //int u=0,v=0;
    Mat test;
    Mat subMat,sub;
    int mocnina = pow(velkost,2);
    this->mat.copyTo(test);

    int w = 0;
    int q = 0;
    Scalar s;
    for(int i = (velkost)/2; i< this->mat_po_normal.rows - (velkost)/2;i++){

        if(i%velkost == 0) w++;

        for(int j = (velkost)/2; j< this->mat_po_normal.cols - (velkost)/2;j++){

           if(j%velkost == 0) q++;
           if((this->maska.at<uchar>(i-(velkost - 1) / 2,j-(velkost - 1) / 2)!=0)&&(this->maska.at<uchar>(i+(velkost - 1) / 2,j+(velkost - 1) / 2)!=0)){ //ak maska na pozici nema ciernu hodnotu
            if(q == (this->Theta.cols))q--;
                 kernel = getGaborKernel(Size(velkost, velkost),4/*this->frekvencnaMat.at<double>(i,j)*/, this->Theta.at<double>(w,q),8/*this->frekvencnaMat.at<double>(i,j)*/,1, 0, CV_64F); //vytvori sa maska Gabora
           subMat = test(Rect(j - velkost/2,i - velkost/2,velkost,velkost));
           subMat.convertTo(sub,CV_64F);
           multiply(sub,kernel,sub);
           s =cv::sum(sub);
           sum = s.val[0] / mocnina;
        this->mat_filtrovana.at<uchar>(i, j) = sum; //zapisem nove hodnoty po filtracii
       sum = 0;
        }
       }
       q = 0;
    }
    ui->loading->setText("");
}

void Predspracovanie::thinning(Mat& img, Mat filtrovana){
    filtrovana.copyTo(img);
    img /= 255;
    Mat prev = Mat::zeros(img.size(), CV_8UC1);
    Mat diff;
    int i=0;
    do{
        thinningIteration(img, 0);
        thinningIteration(img, 1);
        absdiff(img, prev, diff);
        img.copyTo(prev);
    }while(countNonZero(diff) > 0);

    img *=255;
   bitwise_not(this-> kostra, this->kostra);
}

void Predspracovanie::thinningIteration(Mat& img, int iter){
    Mat marker = Mat::zeros(img.size(), CV_8UC1);

    for(int i=1; i<img.rows-1; i++){
        for(int j=1; j<img.cols-1; j++){
            if(this->maska.at<uchar>(i, j)!=0){
            uchar p2 = img.at<uchar>(i-1, j);
            uchar p3 = img.at<uchar>(i-1, j+1);
            uchar p4 = img.at<uchar>(i, j+1);
            uchar p5 = img.at<uchar>(i+1, j+1);
            uchar p6 = img.at<uchar>(i+1, j);
            uchar p7 = img.at<uchar>(i+1, j-1);
            uchar p8 = img.at<uchar>(i, j-1);
            uchar p9 = img.at<uchar>(i-1, j-1);

            int A = (p2 == 0 && p3 ==1)+(p3 == 0 && p4 == 1)+(p4 == 0 && p5 == 1)+(p5 == 0 && p6 == 1)+(p6 == 0 && p7 == 1)+(p7 == 0 && p8 == 1)+(p8 == 0 && p9 == 1)+(p9 == 0 && p2 == 1);

            int B = p2+ p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if(A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0){
                marker.at<uchar>(i,j) = 1;
            }
            }
        }

    }
    img &= ~marker;
}

void Predspracovanie::UpdateView(Mat mat){ //update GUI vpravo
    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView_2);
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView_2->setScene(scn);
    this->item = new QGraphicsPixmapItem(this->Mat2QPixmap(mat));
    scn->addItem(this->item);
    ui->graphicsView_2->show();
}

void Predspracovanie::UpdateView0(Mat mat){ //Update GUI vlavo
    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView);
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView->setScene(scn);
    this->item = new QGraphicsPixmapItem(this->Mat2QPixmap(mat));
    scn->addItem(this->item);
    ui->graphicsView->show();
}

QGraphicsPixmapItem* Predspracovanie::on_pushButton_clicked(QString subor) //otvaranie suboru
{

    this->fingerprint = QPixmap(subor);
    QGraphicsPixmapItem *item1 = new QGraphicsPixmapItem(this->fingerprint);
    return item1;

}

void Predspracovanie::on_pushButton_6_clicked()//500ppi
{
    QElapsedTimer timer;
    timer.start();

    this->calculateOrientations(this->mat,globBlockSize);

    this->adaptive_segmentation(this->mat,globBlockSize,0.161,-0.5,0.975,0.009);
    mat_po_normal = this->normalisation(this->mat,20,5);
    binarizacia(this->mat_po_normal);
    this->calculateOrientations(this->mat_po_normal,globBlockSize);
    this->gaborFilter(globBlockSize);
    binarizacia(this->mat_filtrovana);
    thinning(this->kostra,this->mat_filtrovana);
    bitwise_not(this-> mat_filtrovana, this->mat_filtrovana);

    ui->cas->setText(QString::number(timer.elapsed()) + "ms");
}

