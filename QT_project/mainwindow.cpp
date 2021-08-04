#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QHostAddress>
#include <QRegularExpression>
#include <QMessageBox>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _socket(this),
    flag_connect(false)
{
    ui->setupUi(this);

    connect(&_socket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
    connect(&_socket, SIGNAL(connected()), this, SLOT(onConnected()));
    connect(&_socket, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
    connect(ui->pushButton_enter_ip, SIGNAL(clicked()), this, SLOT(onConnecting()));
    connect(ui->pushButton_change_time_date, SIGNAL(clicked()), this, SLOT(onChange_time_date()));
    connect(ui->pushButton_change_time_average, SIGNAL(clicked()), this, SLOT(onChange_time_average()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onReadyRead()
{
    QByteArray data = _socket.readAll();
    qDebug() << data;
    QString str_data(data);

    QRegExp rexp("^;([+-]?([0-9]*[.])?[0-9]+):([+-]?([0-9]*[.])?[0-9]+):([+-]?([0-9]*[.])?[0-9]+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+);$");
    if (str_data.contains(rexp))
    {
        ui->label_humidity->setText(QString("Humidity: %1 %").arg(rexp.capturedTexts()[1]));
        ui->label_temperature->setText(QString("Temperature: %1 C").arg(rexp.capturedTexts()[3]));
        ui->label_adc->setText(QString("ADC Voltage: %1 V").arg(rexp.capturedTexts()[5]));

        ui->label_date->setText(QString("year: %1   mounth: %2   day: %3   week day: %4").arg(
                                    rexp.capturedTexts()[7],
                                    rexp.capturedTexts()[8],
                                    rexp.capturedTexts()[9],
                                    rexp.capturedTexts()[10]));

        ui->label_time->setText(QString("hours: %1   minutes: %2   seconds: %3").arg(
                                    rexp.capturedTexts()[11],
                                    rexp.capturedTexts()[12],
                                    rexp.capturedTexts()[13]));
        ui->label_probe_time->setText(QString("Probe time: %1 sec").arg(rexp.capturedTexts()[14]));
        ui->label_average->setText(QString("Average: %1 units").arg(rexp.capturedTexts()[15]));
    }
}

void MainWindow::onConnecting()
{
    QRegExp rexp("^(\\d+\\.\\d+\\.\\d+\\.\\d+):(\\d+)$");
    QString str = ui->lineEdit_enter_ip->text();
    bool ok = str.contains(rexp);

    if (ok && flag_connect == false)
    {
        QString IP = rexp.capturedTexts()[1];
        QString port = rexp.capturedTexts()[2];
        qDebug() << IP;
        qDebug() << port;

        ui->lineEdit_enter_ip->setText("connecting...");
        _socket.connectToHost(QHostAddress(IP), (qint16)port.toInt());
    }
    else if (ok == false && flag_connect == false)
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");
        msgBox.setText("Wrong value");
        msgBox.setInformativeText("Not correct input.\n\rFor example 192.168.1.12:12345");
        msgBox.exec();
    }
    else if (this->flag_connect)
    {
        _socket.close();
    }
}

void MainWindow::onConnected()
{
    this->flag_connect = true;

    ui->lineEdit_enter_ip->setText("connected");
    ui->pushButton_enter_ip->setText("disconnect");
}

void MainWindow::onDisconnected()
{
    this->flag_connect = false;

    ui->lineEdit_enter_ip->setText("disconnected");
    ui->pushButton_enter_ip->setText("connect");
}


void MainWindow::onChange_time_date()
{
    QRegExp rexp("^(\\d+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+):(\\d+)$");
    QString str = ui->lineEdit_change_time_date->text();
    bool ok = true;

    if (str.contains(rexp))
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");

        if (!(0 <= rexp.capturedTexts()[1].toInt()) || !(rexp.capturedTexts()[1].toInt() <= 99))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("0 <= year <= 99");
            msgBox.exec();
        }
        else if (!(1 <= rexp.capturedTexts()[2].toInt()) || !(rexp.capturedTexts()[2].toInt() <= 12))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("1 <= mounth <= 12");
            msgBox.exec();
        }
        else if (!(1 <= rexp.capturedTexts()[3].toInt()) || !(rexp.capturedTexts()[3].toInt() <= 31))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("1 <= day <= 31");
            msgBox.exec();
        }
        else if (!(1 <= rexp.capturedTexts()[4].toInt()) || !(rexp.capturedTexts()[4].toInt() <= 7))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("1 <= week day <= 7");
            msgBox.exec();
        }
        else if (!(0 <= rexp.capturedTexts()[5].toInt()) || !(rexp.capturedTexts()[5].toInt() <= 23))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("0 <= hours <= 23");
            msgBox.exec();
        }
        else if (!(0 <= rexp.capturedTexts()[6].toInt()) || !(rexp.capturedTexts()[6].toInt() <= 59))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("0 <= minutes <= 59");
            msgBox.exec();
        }
        else if (!(0 <= rexp.capturedTexts()[7].toInt()) || !(rexp.capturedTexts()[7].toInt() <= 59))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("0 <= seconds <= 59");
            msgBox.exec();
        }

        if (ok && flag_connect)
        {
            _socket.write(str.toUtf8(), str.length());
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");
        msgBox.setText("Wrong value");
        msgBox.setInformativeText("year:mounth:day:week day:hours:minutes:seconds\r\nFor example 21:1:14:5:15:47:30");
        msgBox.exec();
    }
}

void MainWindow::onChange_time_average()
{
    QRegExp rexp("^(\\d+):(\\d+)$");
    QString str = ui->lineEdit_change_time_average->text();
    bool ok = true;

    if (str.contains(rexp))
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");

        if (!(1 <= rexp.capturedTexts()[1].toInt()) || !(rexp.capturedTexts()[1].toInt() <= 1200))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("1 <= seconds <= 1200");
            msgBox.exec();
        }
        else if (!(3 <= rexp.capturedTexts()[2].toInt()) || !(rexp.capturedTexts()[2].toInt() <= 1000 * rexp.capturedTexts()[1].toInt()))
        {
            ok = false;
            msgBox.setText("Wrong value");
            msgBox.setInformativeText("1 <= average <= 1000 * seconds");
            msgBox.exec();
        }

        if (ok && flag_connect)
        {
            str = "a" + str;
            _socket.write(str.toUtf8(), str.length());
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");
        msgBox.setText("Wrong value");
        msgBox.setInformativeText("seconds:average\r\nFor example 3:10");
        msgBox.exec();
    }
}






