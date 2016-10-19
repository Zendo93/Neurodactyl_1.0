#include "advanced.h"
#include "ui_advanced.h"
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QRgb>
#include <QVector>
#include <fstream>
#include <iostream>
#include <doublefann.h>
#include <QMessageBox>
#include <set>
#include "persistence1d.h"
#include <QDebug>
#include <QElapsedTimer>
#include "Graphics_view_zoom.h"


//////////////////
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include <QFileDialog>

#include <QGraphicsTextItem>
#include <QTextStream>
#include <QScrollBar>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QElapsedTimer>
#include <QDebug>
#include <QTextStream>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QColor>
#include <stdio.h>
#include <doublefann.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>




Advanced::Advanced(QWidget *parent) :
    QDialog(parent,Qt::WindowTitleHint | Qt::WindowCloseButtonHint),
    ui(new Ui::Advanced)
{    
    QPixmap pix ("graphics/neurons2.jpg");
    ui->setupUi(this);

    if (!pix.isNull())
    {
        ui->label_20->setPixmap(pix);
        ui->Neuro->setPixmap(pix);
        ui->Neuro_M->setPixmap(pix);
        ui->label_18->setPixmap(pix);
        ui->label_19->setPixmap(pix);
    }

    p = new Predspracovanie(this);
    ui->progressBar->setMinimum(0);
    ui->progressBar->setMaximum(100);
    ui->progressBar->reset();
    ui->progressBar->show();
    ui->progressBar_2->setMinimum(0);
    ui->progressBar_2->setMaximum(100);
    ui->progressBar_2->reset();
    ui->progressBar_2->show();
    ui->tableWidget->setRowCount(3);
    ui->tableWidget->setColumnCount(2);
    ui->tableWidget->setColumnWidth(0,120);
    QStringList header;
    header<<"Typ markantu" << "Rozpoznaných";
    ui->tableWidget->setHorizontalHeaderLabels(header);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
}

Advanced::~Advanced()
{
    delete ui;
}

QPixmap Advanced::Mat2QPixmap(const Mat &matica){   //Opacna konverzia OpenCV matice na maticu umoznujucu zobrazenie v Grafickom prostredi
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

QPixmap Advanced::Mat2QPixmap_RGB(const Mat &src){
   Mat mat;
   cvtColor(src, mat,CV_BGR2RGB);
   QImage img(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
   return QPixmap::fromImage(img);
}

void Advanced::UpdateView(Mat mat){ //update GUI vpravo
    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView_2);
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView_2->setScene(scn);
    this->item = new QGraphicsPixmapItem(this->Mat2QPixmap(mat));
    scn->addItem(this->item);
    ui->graphicsView_2->show();
}

void Advanced::UpdateView0(Mat mat){ //Update GUI vlavo
    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView);
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView->setScene(scn);
    this->item = new QGraphicsPixmapItem(this->Mat2QPixmap(mat));
    scn->addItem(this->item);
    ui->graphicsView->show();
}
void Advanced::on_open_clicked()
{
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView);
    QString subor=QFileDialog::getOpenFileName(
                this,
                tr("Open File"),
                PROJECT_PATH,
                "All files (*.*);;Picture File (*.bmp;*.jpg;*.jpeg;*.gif;*.png;*.tif)"
                );
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView->setScene(scn);
    scn->addItem(p->on_pushButton_clicked(subor));
    ui->graphicsView->show();
    p->PixMap2Mat(p->fingerprint);
}

void Advanced::on_comboBox_activated(const QString &arg1) //vyber roznych faz predspracovania na zobrazenie
{
    if(arg1=="Original"){
        this->UpdateView(p->mat);
    }
    if(arg1=="Po segmentacii"){
        this->UpdateView(p->mat_po_segmentacii);
    }
    if(arg1=="Maska"){
        this->UpdateView(p->maska);
    }
    if(arg1=="Po normalizacii"){
        this->UpdateView(p->mat_po_normal);
    }
    if(arg1=="Smerova mapa"){
        this->UpdateView(p->orientation);
    }
    if(arg1=="Po filtracii"){
        this->UpdateView(p->mat_filtrovana);
    }
    if(arg1=="Kostra"){
        this->UpdateView(p->kostra);
    }
    if(arg1=="Crossing Number"){
        Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView_2);
        QGraphicsScene *scn = new QGraphicsScene();
        ui->graphicsView_2->setScene(scn);
        this->item = new QGraphicsPixmapItem(this->Mat2QPixmap_RGB(p->kostra_signs));
        scn->addItem(this->item);
        ui->graphicsView_2->show();
    }
}

void Advanced::on_comboBox_2_activated(const QString &arg1)
{
    if(arg1=="Original"){
        this->UpdateView0(p->mat);
    }
    if(arg1=="Po segmentacii"){
        this->UpdateView0(p->mat_po_segmentacii);
    }
    if(arg1=="Maska"){
        this->UpdateView0(p->maska);
    }
    if(arg1=="Po normalizacii"){
        this->UpdateView0(p->mat_po_normal);
    }
    if(arg1=="Smerova mapa"){
        this->UpdateView0(p->orientation);
    }
    if(arg1=="Po filtracii"){
        this->UpdateView0(p->mat_filtrovana);
    }
    if(arg1=="Kostra"){
        this->UpdateView0(p->kostra);
    }
    if(arg1=="Crossing Number"){
        Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView);
        QGraphicsScene *scn = new QGraphicsScene();
        ui->graphicsView->setScene(scn);
        this->item = new QGraphicsPixmapItem(this->Mat2QPixmap_RGB(p->kostra_signs));
        scn->addItem(this->item);
        ui->graphicsView->show();
    }
}

void Advanced::on_preprocessButton_clicked()
{
    p->on_pushButton_6_clicked();
    p->kostra_signs = crossingNum(p->kostra);
    this->UpdateView(p->mat_filtrovana);
}

void Advanced::search(Mat img) {
    this->positions.clear();
    QVector <QVector<int> > all;
    int cn,row_shift,col_shift;
    Scalar s;
    int suma;
    cn = 0;
    row_shift = img.rows/10;
    col_shift = img.cols/10;

    for(int k=row_shift; k<img.rows-row_shift; k++) {
        for(int l=col_shift; l<img.cols-col_shift; l++) {
            if(img.at<uchar>(k,l) == 0) {
                cn = abs((img.at<uchar>(k-1,l-1))-(img.at<uchar>(k,l-1)))+abs((img.at<uchar>(k,l-1))-(img.at<uchar>(k+1,l-1)))+abs((img.at<uchar>(k+1,l-1))-(img.at<uchar>(k+1,l)))+abs((img.at<uchar>(k+1,l))-(img.at<uchar>(k+1,l+1)))+abs((img.at<uchar>(k+1,l+1))-(img.at<uchar>(k,l+1)))+abs((img.at<uchar>(k,l+1))-(img.at<uchar>(k-1,l+1)))+abs((img.at<uchar>(k-1,l+1))-(img.at<uchar>(k-1,l)))+abs((img.at<uchar>(k-1,l))-(img.at<uchar>(k-1,l-1)));
                if((cn/255/2 == 1) || (cn/255/2 == 3)) {
                    QVector<int> tmp(3);
                    tmp[0] = cn/255/2;
                    tmp[1] = k;
                    tmp[2] = l;
                    all.push_back(tmp);
                 }
            }
        }
    }
    //filtrovanie markantov nachadzajucich sa po okrajoch
    this->mask_sum = 27*27*255;
    for(int i=0;i<all.size();i++){
        Mat submask = p->maska(Rect(all[i][2]-13,all[i][1]-13,27,27));
        submask.convertTo(submask,CV_64F);

        s = cv::sum(submask);
        suma = s.val[0];
        if(suma == this->mask_sum){
            this->positions.push_back(all[i]);
        }

    }
}

void Advanced::draw_sign(Mat img) {
    int ending=0;
    int bif=0;
    for(int i=0; i<this->positions.size(); i++) {
        Point center(this->positions[i][2],this->positions[i][1]);
        if(this->positions[i][0]==1) {            //ukoncenia
            circle(img,center,3,Scalar(255,0,0),1,8,0);
            ending++;
        } else if(this->positions[i][0]==3) {     //rozdvojenia
            circle(img,center,6,Scalar(0,255,0),1,8,0);
            bif++;
        }
    }
    std::cout << "ending: " << ending << std::endl;
    std::cout << "bifurcation: " << bif << std::endl;
}

Mat Advanced::crossingNum(Mat img) {
    Mat tmp;
    img.copyTo(tmp);
    search(tmp);
    cvtColor(tmp, tmp, CV_GRAY2BGR);
    draw_sign(tmp);

    return tmp;
}

void Advanced::on_comboBox_4_activated(int index)
{
    ui->learn_info->clear();
    ui->progressBar->setValue(0);
    switch (index) {
        case 1:
            ui->block_size->setText(QString::number(11)+" = 121 neurónov");
            break;

        /*case 2:
            ui->block_size->setText(QString::number(51)+" = 2601 neurónov");
            break;*/
        default :
            break;
    }
}

void Advanced::on_comboBox_3_activated(int index)
{
    switch (index) {
        case 1:
            ui->hidden_n_2->setDisabled(true);
            break;
        case 2:
            ui->hidden_n_2->setDisabled(false);
            break;
        default :
            break;
    }
}

void Advanced::on_learn_NN_clicked()
{
    int num_input,num_output;
    int num_layers = 3;
    Mat test_dat;
    String file;
    struct fann *ann;
    ui->progressBar->reset();

    //vytvorenie NN
    if(ui->comboBox_4->currentIndex()==1) {
        num_input = 11*11;
        num_output = 2;
    }
    /*if(ui->comboBox_4->currentIndex()==2) {
        num_input = 51*51;
        num_output = 3;
    }*/

    const unsigned int num_neurons_hidden1 = ui->hidden_n_1->text().toInt();

    const float desired_error = ui->error->text().toFloat();
    const unsigned int max_epochs = ui->epochs_max->text().toInt();
    const unsigned int epochs_between_reports = 20;

    if(ui->comboBox_3->currentIndex()==2) {
        num_layers = 4;
        const unsigned int num_neurons_hidden2 = ui->hidden_n_2->text().toInt();
        ann = fann_create_standard(num_layers, num_input,num_neurons_hidden1,num_neurons_hidden2,num_output);
    } else if(ui->comboBox_3->currentIndex()==1) {
        num_layers = 3;
        ann = fann_create_standard(num_layers, num_input,num_neurons_hidden1,num_output);
    }

    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

    //vyber priecinka s test. datami
    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),"",
                                                 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    //otvorenie zvoleneho priecinka a nacitanie suborov
    dir.setNameFilters(QStringList()<<"*.bmp");
    QFileInfoList list = dir.entryInfoList();
    //ak priecinok obsahuje nejake bmp subory
    if(list.size()!=0){
        //vytvorenie strukt. na testovacie data
        fann_train_data *data;
        int value,percent;

        //fann_create_train(pocet vzoriek, pocet_vstupov, pocet_vystupov)
        data = fann_create_train(list.length(),num_input,num_output);
        double step = 100/list.length();
        for(int k=0;k<list.length();k++) {
            percent = ui->progressBar->value();
            ui->progressBar->setValue(percent+step);
            file = list[k].absoluteFilePath().toStdString();
            test_dat = imread(file, CV_LOAD_IMAGE_GRAYSCALE);   //nacitanie test. suboru do mat
            int c = 0;                                          //porad. cislo pixelu

            //postupne nacitanie trenovacich suborov
            for(int i=0;i<test_dat.rows;i++) {
                for(int j=0;j<test_dat.cols;j++) {
                    value =(int)test_dat.at<uchar>(i,j);
                    if(value == 255){
                        value = 1;
                    }
                    if(value == 0) {
                        value = -1;
                    }
                    data->input[k][c] = value;          //zapis hodnot pixelov do vektora
                    c++;
                }
            }
            if(ui->comboBox_4->currentIndex()==1) {
                if(file.find("Ukoncenie") != string::npos) {
                    data->output[k][0] = 1;
                    data->output[k][1] = -1;
                } else if(file.find("Vidlica") != string::npos) {
                    data->output[k][0] = -1;
                    data->output[k][1] = 1;
                }
            }
        }
        String train_file,network_file;
        if(ui->comboBox_4->currentIndex()==1) {
            train_file = "neural_networks/basic.data";
            network_file = "neural_networks/basic.net";
        } /*else if(ui->comboBox_4->currentIndex()==2){
            train_file = "complex.data";
            network_file = "complex.net";
        }*/
        //ulozenie tren. dat do suboru
        fann_save_train(data,train_file.c_str());

        //trenovanie NS
        ui->learn_info->setText("Trenovanie siete je zahájené.");
        fann_train_on_data(ann, data, max_epochs, epochs_between_reports, desired_error);
        //ulozenie natrenovanej NS do suboru
        fann_save(ann, network_file.c_str());
        fann_destroy(ann);
        ui->progressBar->setValue(100);
        ui->learn_info->setText("Trénovanie siete prebehlo úspešne.");
    } else {
        ui->learn_info->setText("Nebol zvolený vhodný priečinok.");
    }
}


void Advanced::on_learn_global_clicked()
{
    int num_input,num_output;
    int num_layers = 3;
    float conn_rate = 0.3;
    float learn_rate = 0.7;
    float steepness = 0.1;
    Mat test_dat;
    String file;
    struct fann *ann;
    ui->progressBar_2->reset();

    num_input = 812;
    num_output =  5;

    const unsigned int num_neurons_hidden1 = ui->global_hidden->text().toInt();

    const float desired_error = ui->global_error->text().toFloat();
    const unsigned int max_epochs = ui->global_epochs->text().toInt();
    const unsigned int epochs_between_reports = 20;

    learn_rate = ui->global_learn_rate->text().toFloat();
    conn_rate = ui->global_conn_rate->text().toFloat();
    steepness = ui->global_stepness->text().toFloat();

    ann = fann_create_sparse(conn_rate,num_layers,num_input,num_neurons_hidden1,num_output);

    //nastavenie neuronovej siete
    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);
    fann_set_learning_rate(ann, learn_rate);
    fann_set_activation_steepness_hidden(ann, 0.5);
    fann_set_activation_steepness_output(ann, steepness);
    fann_set_train_error_function(ann,FANN_ERRORFUNC_LINEAR);
    fann_set_learning_momentum(ann, 0.0);

    //vyber priecinka s test. datami
    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),"",
                                                 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    //otvorenie zvoleneho priecinka a nacitanie suborov

    dir.setNameFilters(QStringList()<<"*.bmp");
    QFileInfoList list = dir.entryInfoList();

    if(list.size()!=0){
        //vytvorenie strukt. na testovacie data
        fann_train_data *data;
        int value,percent;

        //fann_create_train(pocet vzoriek, pocet_vstupov, pocet_vystupov)
        data = fann_create_train(list.length(),num_input,num_output);
        double step = 100/list.length();
        for(int k=0;k<list.length();k++) {
            percent = ui->progressBar_2->value();
            ui->progressBar_2->setValue(percent+step);
            file = list[k].absoluteFilePath().toStdString();
            test_dat = imread(file, CV_LOAD_IMAGE_GRAYSCALE);   //nacitanie test. suboru do mat

            p->on_pushButton_clicked(QString::fromStdString(file));
            p->PixMap2Mat(p->fingerprint);

            p->calculateOrientations(p->mat,p->globBlockSize);
            p->adaptive_segmentation(p->mat,p->globBlockSize,0.161,-0.5,0.975,0.009);
            p->mat_po_normal = p->normalisation(p->mat,20,5);
            p->binarizacia(p->mat_po_normal);
            p->calculateOrientations(p->mat_po_normal,p->globBlockSize);

            //postupne nacitanie trenovacich suborov
            for(int i=0;i<812;i++) {
                    data->input[k][i] = p->fann_smery[i];          //zapis hodnot pixelov do vektora
            }

            if(file.find("arch") != string::npos) {
                data->output[k][0] = 1;
                data->output[k][1] = 0.25;
                data->output[k][2] = 0.25;
                data->output[k][3] = 0;
                data->output[k][4] = 0;
            } else if(file.find("double") != string::npos) {
                data->output[k][0] = 0;
                data->output[k][1] = 0;
                data->output[k][2] = 0;
                data->output[k][3] = 0;
                data->output[k][4] = 1;
            } else if(file.find("left") != string::npos) {
                data->output[k][0] = 0.25;
                data->output[k][1] = 1;
                data->output[k][2] = 0;
                data->output[k][3] = 0;
                data->output[k][4] = 0;
            } else if(file.find("right") != string::npos) {
                data->output[k][0] = 0.25;
                data->output[k][1] = 0;
                data->output[k][2] = 1;
                data->output[k][3] = 0;
                data->output[k][4] = 0;
            } else if(file.find("whorl") != string::npos) {
                data->output[k][0] = 0;
                data->output[k][1] = 0;
                data->output[k][2] = 0;
                data->output[k][3] = 0;
                data->output[k][4] = 1;
            }
        }

        String train_file,network_file;
        train_file = "neural_networks/global.data";
        network_file = "neural_networks/global.net";

        //ulozenie tren. dat do suboru
        fann_save_train(data,train_file.c_str());

        //trenovanie NS
        ui->global_info->setText("Trenovanie siete je zahájené.");
        fann_train_on_data(ann, data, max_epochs, epochs_between_reports, desired_error);
        //ulozenie natrenovanej NS do suboru
        fann_save(ann, network_file.c_str());
        fann_destroy(ann);
        ui->progressBar_2->setValue(100);
        ui->global_info->setText("Trénovanie siete prebehlo úspešne.");
    } else {
        ui->global_info->setText("Nebol zvolený vhodný priečinok.");
    }
}


void Advanced::on_comboBox_5_activated(int index)
{
    ui->extract->setDisabled(false);
    ui->extract_info->clear();
    ui->progressBar_3->setValue(0);
    ui->tableWidget->clearContents();

    switch (index) {
        case 0:
            ui->extract_info->setText("");
        case 1:
            this->network_file = "neural_networks/basic.net";
            ui->tableWidget->setItem(0, 0, new QTableWidgetItem("Ukončenia"));
            ui->tableWidget->setItem(1, 0, new QTableWidgetItem("Rozdvojenia"));
            break;
        case 2:
            this->network_file = "neural_networks/complex.net";
            ui->tableWidget->setItem(0, 0, new QTableWidgetItem("Prerušenia"));
            ui->tableWidget->setItem(1, 0, new QTableWidgetItem("Premostenia"));
            ui->tableWidget->setItem(2, 0, new QTableWidgetItem("Protiľahlé rozdvojenia"));
            break;
        default :
            break;
    }

    //kontrola existencie suboru s naucenou NS
    QFile file(QString::fromStdString(this->network_file));
    if(!file.exists()) {
        ui->extract_info->setText("Pre zvolený typ markantov nebola ešte NS natrénovaná.");
        ui->extract->setDisabled(true);
    }
}

Mat Advanced::draw_detected(Mat img, QVector <minutiae> detected) {
    cvtColor(img, img, CV_GRAY2BGR);
    for(int i=0; i<detected.size(); i++) {
        Point center(detected[i].y,detected[i].x);
        if(detected[i].type==1) {            //ukoncenia
            circle(img,center,3,Scalar(255,0,0),1,8,0);
        } else if(detected[i].type==2) {     //rozdvojenia
            circle(img,center,6,Scalar(0,0,255),1,8,0);
        } else if(detected[i].type==11) {     //prerusenia
            circle(img,center,15,Scalar(0,0,255),1,8,0);
        } else if(detected[i].type==12) {     //premostenia
            circle(img,center,15,Scalar(0,255,0),1,8,0);
        } else if(detected[i].type==13) {     //protilahle rozdvojenia
            circle(img,center,10,Scalar(255,0,0),1,8,0);
        }
    }

    return img;
}

bool Advanced::give_global(){
    struct fann *ann;
    vector<double> euklid;
    vector<double> out(5);
    vector<vector<double>> classes(4);
    classes[0] = {1.0, 0.25, 0.25, 0.0, 0.0};
    classes[1] = {0.25, 1.0, 0.0, 0.0, 0.0};
    classes[2] = {0.25, 0.0, 1.0, 0.0, 0.0};
    classes[3] = {0.0, 0.0, 0.0, 0.0, 1.0};
    this->global_net = "neural_networks/global.net";


    QFileInfo check_file(QString::fromStdString(this->global_net));
    if (check_file.exists() && check_file.isFile()) {
        ann = fann_create_from_file(this->global_net.c_str());
        double *output = fann_run(ann, p->fann_smery);
        for(int i=0;i<5;i++){
            out[i] = output[i];
        }

        for(int i=0;i<4;i++){
            euklid.push_back(norm(classes[i],out));
        }

        int dist = distance(euklid.begin(),std::min_element(euklid.begin(),euklid.end()));
        this->global_class = dist;
        return true;
    } else {
        return false;
    }
}

void Advanced::on_extract_clicked()
{
    struct fann *ann;
    minutiae tmp;
    int value;
    ui->extract_info->clear();
    ui->progressBar_3->reset();
    this->detected_minutiae.clear();
    int end = 0;
    int bif = 0;
    int prerus = 0;
    int bridge = 0;
    int proti = 0;
    QElapsedTimer timer;
    timer.start();

    QFile outputFile1(basic_filename);
    outputFile1.open(QIODevice::WriteOnly);
    QTextStream outBasic(&outputFile1);

    //vytvorenie NS zo suboru
    if(this->network_file.empty()) {
        ui->extract_info->setText("Nebol zvolený typ markantov pre extrakciu.");
    } else {
        ann = fann_create_from_file(this->network_file.c_str());

        if(!this->positions.empty()) {
            int step = 100/positions.size();
            for(int i=0; i<this->positions.size(); i++) {               //kontrola vsetkych suradnic poskytnutych CN
                value = ui->progressBar_3->value();
                ui->progressBar_3->setValue(value+step);
                if(this->positions[i][0]==1) {                          //ukoncenia detekovane CN
                    if(ui->comboBox_5->currentIndex()==1) {             //extrakcia zakladnych markantov (ukoncenia)
                        double test[121];
                        int pix = 0;
                        for(int k=(this->positions[i][1])-5; k<=(this->positions[i][1])+5; k++) {
                            for(int l=(this->positions[i][2])-5; l<=(this->positions[i][2])+5; l++) {
                                if((int)p->mat_filtrovana.at<uchar>(k,l)==255) {
                                    test[pix] = 1;
                                } else {
                                    test[pix] = -1;
                                }
                                pix++;
                            }
                        }
                        timer.restart();
                        double *output = fann_run(ann, test);

                        //pridanie spravne rozpoznaneho markantu do vystupneho vektora
                        if(output[0]>0.9) {
                            tmp.type = 1;   //ukoncenie
                            tmp.x = this->positions[i][1];
                            tmp.y = this->positions[i][2];
                            this->detected_minutiae.push_back(tmp);
                            end++;
                        }
                        output = NULL;
                    } /*else if (ui->comboBox_5->currentIndex()==2) {     //extrakcia komplexnych markantov (prerusenie)
                        if (((this->positions[i][1]-25) >= 0)&&
                            ((this->positions[i][2]-25) >= 0)&&
                            ((this->positions[i][1]+25) < p->mat_filtrovana.rows)&&
                            ((this->positions[i][2]+25) < p->mat_filtrovana.cols)) {
                            double test[2601];
                            int pix = 0;                                    //nacitanie bloku obrazu okolo suradnice
                            for(int k=(this->positions[i][1])-25; k<=(this->positions[i][1])+25; k++) {
                                for(int l=(this->positions[i][2])-25; l<=(this->positions[i][2])+25; l++) {
                                    if((int)p->mat_filtrovana.at<uchar>(k,l)==255) {
                                        test[pix] = 1;
                                    } else {
                                        test[pix] = -1;
                                    }
                                    pix++;
                                }
                            }

                            timer.restart();
                            double *output = fann_run(ann, test);

                            //pridanie spravne rozpoznaneho markantu do vystupneho vektora
                            if((output[0]>0.98) && (output[1]<-0.8) && (output[2]<-0.8)) {
                                prerus++;
                                tmp.type = 11;   //prerusenie
                                tmp.x = this->positions[i][1];
                                tmp.y = this->positions[i][2];
                                this->detected_minutiae.push_back(tmp);
                            }
                            cout << "prerusenie : " << i << " " << output[0]  << " " << output[1] << " " << output[2] << endl;
                        }

                    } */
                } else if(this->positions[i][0]==3) {
                    if(ui->comboBox_5->currentIndex()==1) {             //extrakcia zakladnych markantov (rozdvojeni)
                        double test[121];
                        int pix = 0;
                        for(int k=(this->positions[i][1])-5; k<=(this->positions[i][1])+5; k++) {
                            for(int l=(this->positions[i][2])-5; l<=(this->positions[i][2])+5; l++) {
                                if((int)p->mat_filtrovana.at<uchar>(k,l)==255) {
                                    test[pix] = 1;
                                } else {
                                    test[pix] = -1;
                                }
                                pix++;
                            }
                        }
                        timer.restart();
                        double *output = fann_run(ann, test);

                        //pridanie spravne rozpoznaneho markantu do vystupneho vektora
                        if(output[1]>0.9) {
                            tmp.type = 2;   //rozdvojenie
                            tmp.x = this->positions[i][1];
                            tmp.y = this->positions[i][2];
                            this->detected_minutiae.push_back(tmp);
                            bif++;
                        }
                    } /*else if (ui->comboBox_5->currentIndex()==2) {     //extrakcia protilahlych rozdvojeni a premosteni
                        if (((this->positions[i][1]-25) >= 0)&&
                            ((this->positions[i][2]-25) >= 0)&&
                            ((this->positions[i][1]+25) < this->input.rows)&&
                            ((this->positions[i][2]+25) < this->input.cols)) {
                            double test[2601];
                            int pix = 0;
                            for(int k=(this->positions[i][1])-25; k<=(this->positions[i][1])+25; k++) {
                                for(int l=(this->positions[i][2])-25; l<=(this->positions[i][2])+25; l++) {
                                    if((int)p->mat_filtrovana.at<uchar>(k,l)==255) {
                                        test[pix] = 1;
                                    } else {
                                        test[pix] = -1;
                                    }
                                    pix++;
                                }
                            }

                            timer.restart();
                            double *output = fann_run(ann, test);

                            //pridanie spravne rozpoznaneho markantu do vystupneho vektora
                            if((output[0]<-0.8) && (output[1]>0.98) && (output[2]<-0.8)) {
                                bridge++;
                                tmp.type = 12;   //premostenie
                                tmp.x = this->positions[i][1];
                                tmp.y = this->positions[i][2];
                                this->detected_minutiae.push_back(tmp);
                                cout << "premostenie : " << i << " " << output[0]  << " " << output[1]
                                                                     << " " << output[2] << endl;
                            } else if((output[0]<-0.8) && (output[1]<-0.8) && (output[2]>0.98)) {
                                proti++;
                                tmp.type = 13;   //protilahle rozdvojenie
                                tmp.x = this->positions[i][1];
                                tmp.y = this->positions[i][2];
                                this->detected_minutiae.push_back(tmp);
                                cout << "protilahle rozdvojenia : " << i << " " << output[0]  << " " << output[1]
                                                                                << " " << output[2] << endl;
                            }

                        }
                    }*/
                }
            }
            ui->progressBar_3->setValue(100);
            ui->extract_info->setText("Extrakcia markantov bola dokončená.");

            Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView_3);
            QGraphicsScene *scn = new QGraphicsScene();
            ui->graphicsView_3->setScene(scn);
            Mat extracted;

            p->mat_filtrovana.copyTo(extracted);

            Mat tempMat(extracted.size(),CV_8UC1, Scalar(0));
            for(int i =0; i<this->detected_minutiae.size();i++){
                if(this->detected_minutiae[i].type == 1)
                tempMat.at<uchar>(this->detected_minutiae[i].x,this->detected_minutiae[i].y) = 1;
            }

            for(int i = 0; i < this->detected_minutiae.size(); i++) {
                if(this->detected_minutiae[i].type == 1) {
                    Mat submask = tempMat(Rect(this->detected_minutiae[i].y-10,this->detected_minutiae[i].x-10,21,21));
                    submask.convertTo(submask,CV_64F);
                    Scalar s;
                    int suma;
                    s = cv::sum(submask);
                    suma = s.val[0];
                    if(suma <2){
                        this->real_minutiae.push_back(this->detected_minutiae[i]);
                    }
                }
            }
            tempMat = Mat::zeros(extracted.size(),CV_8UC1);

            for(int i =0; i<this->detected_minutiae.size();i++){
                if(this->detected_minutiae[i].type == 2)
                tempMat.at<uchar>(this->detected_minutiae[i].x,this->detected_minutiae[i].y) = 1;
            }

            for(int i = 0; i < this->detected_minutiae.size(); i++) {
                if(this->detected_minutiae[i].type == 2) {
                    Mat submask = tempMat(Rect(this->detected_minutiae[i].y-10,this->detected_minutiae[i].x-10,21,21));
                    submask.convertTo(submask,CV_64F);
                    Scalar s;
                    int suma;
                    s = cv::sum(submask);
                    suma = s.val[0];
                    if(suma <2){
                        this->real_minutiae.push_back(this->detected_minutiae[i]);
                    }
                }
            }

            this->output_cn = draw_detected(extracted,this->real_minutiae);

            if (ui->comboBox_5->currentIndex()==1) {                            //vypisanie statist. udajov do tabulky
                ui->tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(end)));
                ui->tableWidget->setItem(1, 1, new QTableWidgetItem(QString::number(bif)));
                //imwrite("basic.bmp",this->output_cn);
            } else if (ui->comboBox_5->currentIndex()==2) {
                ui->tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(prerus)));
                ui->tableWidget->setItem(1, 1, new QTableWidgetItem(QString::number(bridge)));
                ui->tableWidget->setItem(2, 1, new QTableWidgetItem(QString::number(proti)));
                //imwrite("complex.bmp",this->output_cn);
            }

            this->item = new QGraphicsPixmapItem(Mat2QPixmap_RGB(this->output_cn));
            scn->addItem(this->item);
            ui->graphicsView_3->show();

            for(int i=0;i<this->real_minutiae.size();i++){
                double theta = p->Theta.at<double>((int)(this->real_minutiae[i].x/17),(int)(this->real_minutiae[i].y/17));
                theta = theta*180/CV_PI;
                if(theta < 0.0){
                    theta += 180.0;
                }
                outBasic << this->real_minutiae[i].x << " " << this->real_minutiae[i].y
                         << " " << (int)theta <<endl;
            }


            outputFile1.close();
            this->detected_minutiae.clear();
            this->positions.clear();
            this->real_minutiae.clear();
            end = 0;
            bif = 0;
            prerus = 0;
            bridge = 0;
            proti = 0;
        } else
            ui->extract_info->setText("Nebol zvolený odtlačok pre extrakciu markantov.");
    }
}

//vytvorenie databazy zo zvoleneho priecinku
void Advanced::on_file_automatic_clicked()
{
    QString file_read;
    ui->comboBox_5->setCurrentIndex(1);

    this->network_file = "neural_networks/basic.net";

    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView_4);
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView_4->setScene(scn);

    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),"",
                                                 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    //otvorenie zvoleneho priecinka a nacitanie suborov
    dir.setNameFilters(QStringList()<<"*.bmp");

    QFileInfoList list = dir.entryInfoList();

    if(list.size()!=0){
        //vytvorenie systemu priecinkov pre ukladanie vysledkov
        QString qs(QDir::currentPath()+"/"+"DB");
        QDir qdir(qs);
        if (!qdir.exists()){
            qdir.mkdir(".");
        }
        qs = qdir.path()+"/"+ dir.dirName();
        QDir qdir2(qs);
        if (!qdir2.exists()){
            qdir2.mkdir(".");
        }
         QElapsedTimer timer;
         timer.start();

        //prechadzanie celeho zoznamu suborov
        for(int i=0;i<list.size();i++){
            file_read = list[i].absoluteFilePath();

            scn->clear();
            scn->addItem(p->on_pushButton_clicked(file_read));
            ui->graphicsView_4->show();

            p->PixMap2Mat(p->fingerprint);

            //vykonanie predspracovania
            this->basic_filename = "temporary/basic.txt";
            this->mask_sum = 27*27*255;
            this->on_preprocessButton_clicked();

            //nachystanie nazvu suboru pre ulozenie
            QString temp = file_read.left(file_read.size()-3);
            int pos = temp.lastIndexOf('/');
            QString filename = qdir2.path()+"/" + temp.right(temp.size()-1-pos) + "xyt";
            this->basic_filename = filename;

            //kontrola existencie suboru s naucenou NS
            QFile file(QString::fromStdString(this->network_file));
            if(!file.exists()) {
                ui->automatic_info->setText("Nebol nájdený súbor s natrénovanou NS.");
            } else {

                this->on_extract_clicked();
                this->item = new QGraphicsPixmapItem(Mat2QPixmap_RGB(this->output_cn));
                scn->addItem(this->item);
                ui->graphicsView_4->show();
            }
            ui->automatic_info->setText("Vytvorenie DB bolo úspešne dokončené.");
        }
    } else {
        ui->automatic_info->setText("Nebol zvolený vhodný priečinok.");
    }

}

void Advanced::on_pushButton_2_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "",
                tr("Jpg files (*.jpg);;bmp files (*.bmp);;tiff files (*.tiff);;png files (*.png);;jp2 files (*.jp2);;All files files (*.*)"));
        if (!fileName.trimmed().isEmpty()) {
            this->item->pixmap().save(fileName);
       }

}
