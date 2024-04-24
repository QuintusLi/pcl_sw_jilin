/********************************************************************************
** Form generated from reading UI file 'visualization.ui'
**
** Created by: Qt User Interface Compiler version 6.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VISUALIZATION_H
#define UI_VISUALIZATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_visualization
{
public:
    QWidget *centralwidget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *visualization)
    {
        if (visualization->objectName().isEmpty())
            visualization->setObjectName("visualization");
        visualization->resize(800, 600);
        centralwidget = new QWidget(visualization);
        centralwidget->setObjectName("centralwidget");
        visualization->setCentralWidget(centralwidget);
        menubar = new QMenuBar(visualization);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 17));
        visualization->setMenuBar(menubar);
        statusbar = new QStatusBar(visualization);
        statusbar->setObjectName("statusbar");
        visualization->setStatusBar(statusbar);

        retranslateUi(visualization);

        QMetaObject::connectSlotsByName(visualization);
    } // setupUi

    void retranslateUi(QMainWindow *visualization)
    {
        visualization->setWindowTitle(QCoreApplication::translate("visualization", "MainWindow", nullptr));
    } // retranslateUi

};

namespace Ui {
    class visualization: public Ui_visualization {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VISUALIZATION_H
