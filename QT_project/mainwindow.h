#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void onReadyRead();
    void onConnected();
    void onDisconnected();
    void onConnecting();
    void onChange_time_date();
    void onChange_time_average();

private:
    Ui::MainWindow *ui;
    QTcpSocket _socket;

    bool flag_connect;
};

#endif // MAINWINDOW_H
