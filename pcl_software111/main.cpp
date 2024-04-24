#include "mainwindow.h"
#include <QApplication>
#include<QMutex>
#include<QTime>
#include<QQueue>
#include<QFile>
#include <QStyleFactory>
void LogMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static QMutex mutex;
    Q_UNUSED(context);
    mutex.lock();
    QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss:zzz");
    QString message = QString("%1  %2\r\n").arg(current_date_time).arg(msg);
    LogElement log_element={type,message};
    log_txt.enqueue(log_element);
    mutex.unlock();
}
int main(int argc, char *argv[])
{


    QApplication a(argc, argv);
    a.setStyle(QStyleFactory::create("windows"));
    qInstallMessageHandler(LogMessage);
    qDebug()<<"startup application!";
    MainWindow w;
    w.show();

    return a.exec();
}

