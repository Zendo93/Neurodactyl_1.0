#include "automatic.h"
#include "ui_automatic.h"
#include "advanced.h"
#include "ui_advanced.h"
#include <QFileDialog>
#include <QGraphicsView>
#include "Graphics_view_zoom.h"
#include <QString>
#include <QStringList>
#include <QFile>
#include <QElapsedTimer>
#include <QDebug>
#include <iostream>
#include <fstream>
#include <QDir>

using namespace std;

Automatic::Automatic(QWidget *parent) :
    QDialog(parent, Qt::WindowTitleHint | Qt::WindowCloseButtonHint),
    ui(new Ui::Automatic)
{
    QPixmap pix ("graphics/neurons2.jpg");
    ui->setupUi(this);

    if (!pix.isNull())
    {
        ui->label->setPixmap(pix);
    }
    adv = new Advanced(this);
}

Automatic::~Automatic()
{
    delete ui;
}

void Automatic::UpdateView(Mat mat){ //update GUI
    Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView);
    QGraphicsScene *scn = new QGraphicsScene();
    ui->graphicsView->setScene(scn);
    this->item = new QGraphicsPixmapItem(adv->Mat2QPixmap_RGB(mat));
    scn->addItem(this->item);
    this->ui->graphicsView->show();
    qApp->processEvents();
}

void Automatic::on_file_automatic_clicked()
{
    ui->automatic_info->clear();
    QString file_read, global;

    QDir dir = "DB";
    if(dir.exists()){

        adv->ui->comboBox_5->setCurrentIndex(1);
        adv->network_file = "neural_networks/basic.net";

        Graphics_view_zoom* zoom = new Graphics_view_zoom(ui->graphicsView);
        QString subor=QFileDialog::getOpenFileName(
                    this,
                    tr("Open File"),
                    PROJECT_PATH,
                    "All files (*.*);;Picture File (*.bmp;*.jpg;*.jpeg;*.gif;*.png;*.tif)"
                    );
        QGraphicsScene *scn = new QGraphicsScene();
        ui->graphicsView->setScene(scn);


         QElapsedTimer timer;
         timer.start();

        if(subor.size()!=0){
            scn->addItem(adv->p->on_pushButton_clicked(subor));
            adv->p->PixMap2Mat(adv->p->fingerprint);

            //vykonanie predspracovania
            adv->mask_sum = 27*27*255;
            adv->on_preprocessButton_clicked();
            //nachystanie nazvu suboru pre ulozenie
            QString temp = subor.left(subor.size()-3);
            int pos = temp.lastIndexOf('/');
            QString filename = "temporary/identified.xyt";
            adv->basic_filename = filename;

            if(adv->give_global()){
                switch(adv->global_class){
                case 0:
                    global = "DB/A/";
                    break;
                case 1:
                    global = "DB/LL/";
                    break;
                case 2:
                    global = "DB/RL/";
                    break;
                case 3:
                    global = "DB/W/";
                    break;
                default:
                    break;
                }

                //kontrola existencie suboru s naucenou NS
                QFile file(QString::fromStdString(adv->network_file));
                if(!file.exists()) {
                    ui->automatic_info->setText("Nebol nájdený súbor s natrénovanou NS.");
                } else {

                    adv->on_extract_clicked();

                    scn->addItem(adv->item);
                    ui->graphicsView->show();
                }

                compare(global);
            } else {
                ui->automatic_info->setText("Neexistuje súbor s natrénovanou neurónovou sieťou.");
            }
        } else{
            ui->automatic_info->setText("Nebol zvolený žiaden súbor pre identifikáciu.");
        }
    } else {
        ui->automatic_info->setText("Neexistuje databáza pre identifikáciu.");
    }
}

void Automatic::compare(QString global){

    QDir dir ("./" + global);
    QStringList list = dir.entryList(QDir::Files);

    QString file = "temporary/identified.xyt";
    QString s("./"); // priecinok kde je bozorth3.exe
    QString s2 = dir.path() + "/";// priecinok kde su subory danej triedy na porovnanie
    QString p = s + "temporary/mates.lis";
    QString t1;

    ofstream ofs(p.toStdString().c_str()); //subor kam ulozim nazvy suborov na porovnanie
    for(int i = 0; i < list.size(); i++) {
        t1 = s2+list[i];
        ofs << file.toStdString().c_str() << endl << t1.toStdString().c_str() << endl;
    }
    ofs.close();

    char command[200];
    QString path("bozorth3.exe -o " + s+"temporary/bozorth_output.txt -M " + p.toStdString().c_str());
    sprintf_s(command, path.toStdString().c_str());
    system(command);

    int val;
    vector<int> outputs;
    ifstream ifs("./temporary/bozorth_output.txt");
    while(ifs >> val) {
        outputs.push_back(val);
    }
    ifs.close();

    int value = distance(outputs.begin(),std::max_element(outputs.begin(),outputs.end()));
    if(outputs[value] > adv->treshold){
        QString person = list[value].left(5);
        ui->automatic_info->setText("Odtlačok identifikovaný, osoba: " + person);
    } else
        ui->automatic_info->setText("Hľadaná osoba sa v databáze nenachádza.");
}
