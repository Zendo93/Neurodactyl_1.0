#include "start_window.h"
#include "ui_start_window.h"
#include <QMovie>
Start_window::Start_window(QWidget *parent) :
    QDialog(parent,Qt::FramelessWindowHint),
    ui(new Ui::Start_window)
{
    QMovie* movie = new QMovie("graphics/Neurodactyl.gif");
    ui->setupUi(this);
    if (movie->isValid())
    {
        // Play GIF
        ui->Neurogif->setMovie(movie);
        movie->start();
    }
}

Start_window::~Start_window()
{
    delete ui;
}

void Start_window::on_advanced_clicked()
{
    this->advanced = new Advanced(this);
    this->advanced->show();
}

void Start_window::on_identification_clicked()
{
    this->advanced = new Advanced(this);
    this->automatic = new Automatic(this);
    this->automatic->show();
}

void Start_window::on_pushButton_clicked()
{
    this->close();
}
