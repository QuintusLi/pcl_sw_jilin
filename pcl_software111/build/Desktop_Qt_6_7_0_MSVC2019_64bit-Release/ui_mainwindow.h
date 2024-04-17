/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QVTKOpenGLNativeWidget.h>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *open_action;
    QAction *save_action;
    QAction *close_action;
    QAction *exit_action;
    QAction *actionPCD;
    QAction *actionPLY;
    QAction *actionTXT;
    QAction *actionOBJ;
    QAction *actionSTL;
    QWidget *centralwidget;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_3;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QVTKOpenGLNativeWidget *guiwidget;
    QMenuBar *menubar;
    QMenu *menu;
    QMenu *menu_output;
    QMenu *menu_2;
    QMenu *menu_3;
    QMenu *menu_4;
    QMenu *menu_5;
    QMenu *menu_6;
    QMenu *menu_7;
    QMenu *menu_8;
    QMenu *menu_9;
    QMenu *menu_10;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QToolBar *toolBar_2;
    QDockWidget *dockWidget_4;
    QWidget *dockWidgetContents_3;
    QVBoxLayout *verticalLayout;
    QTextEdit *textEdit;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents_2;
    QHBoxLayout *horizontalLayout;
    QTextEdit *log_textEdit;
    QDockWidget *dockWidget_2;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QTextBrowser *textBrowser;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_8;
    QWidget *widget_4;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label;
    QGridLayout *gridLayout;
    QPushButton *main_view;
    QPushButton *back_view;
    QPushButton *left_view;
    QPushButton *right_view;
    QPushButton *top_view;
    QPushButton *upward_view;
    QFrame *line;
    QWidget *widget_3;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_2;
    QVBoxLayout *verticalLayout_5;
    QComboBox *comboBox;
    QComboBox *comboBox_2;
    QPushButton *render_begin;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_6;
    QWidget *widget_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QVBoxLayout *verticalLayout_7;
    QPushButton *chazhi;
    QPushButton *chazhi1;
    QFrame *line_2;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_4;
    QPushButton *TaLanSanJiao;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 655);
        open_action = new QAction(MainWindow);
        open_action->setObjectName("open_action");
        open_action->setCheckable(false);
        open_action->setMenuRole(QAction::NoRole);
        save_action = new QAction(MainWindow);
        save_action->setObjectName("save_action");
        save_action->setEnabled(false);
        save_action->setMenuRole(QAction::NoRole);
        close_action = new QAction(MainWindow);
        close_action->setObjectName("close_action");
        close_action->setEnabled(false);
        close_action->setMenuRole(QAction::NoRole);
        exit_action = new QAction(MainWindow);
        exit_action->setObjectName("exit_action");
        exit_action->setEnabled(false);
        exit_action->setMenuRole(QAction::NoRole);
        actionPCD = new QAction(MainWindow);
        actionPCD->setObjectName("actionPCD");
        actionPCD->setEnabled(true);
        actionPLY = new QAction(MainWindow);
        actionPLY->setObjectName("actionPLY");
        actionPLY->setEnabled(true);
        actionPLY->setMenuRole(QAction::NoRole);
        actionTXT = new QAction(MainWindow);
        actionTXT->setObjectName("actionTXT");
        actionTXT->setEnabled(true);
        actionTXT->setMenuRole(QAction::NoRole);
        actionOBJ = new QAction(MainWindow);
        actionOBJ->setObjectName("actionOBJ");
        actionOBJ->setEnabled(true);
        actionOBJ->setMenuRole(QAction::NoRole);
        actionSTL = new QAction(MainWindow);
        actionSTL->setObjectName("actionSTL");
        actionSTL->setEnabled(true);
        actionSTL->setMenuRole(QAction::NoRole);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName("layoutWidget");
        layoutWidget->setGeometry(QRect(0, 0, 100, 30));
        verticalLayout_3 = new QVBoxLayout(layoutWidget);
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName("layoutWidget1");
        layoutWidget1->setGeometry(QRect(0, 0, 100, 30));
        verticalLayout_4 = new QVBoxLayout(layoutWidget1);
        verticalLayout_4->setObjectName("verticalLayout_4");
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout(centralwidget);
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        guiwidget = new QVTKOpenGLNativeWidget(centralwidget);
        guiwidget->setObjectName("guiwidget");

        horizontalLayout_2->addWidget(guiwidget);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 17));
        menu = new QMenu(menubar);
        menu->setObjectName("menu");
        menu_output = new QMenu(menu);
        menu_output->setObjectName("menu_output");
        menu_2 = new QMenu(menubar);
        menu_2->setObjectName("menu_2");
        menu_2->setEnabled(false);
        menu_3 = new QMenu(menubar);
        menu_3->setObjectName("menu_3");
        menu_3->setEnabled(false);
        menu_4 = new QMenu(menubar);
        menu_4->setObjectName("menu_4");
        menu_4->setEnabled(false);
        menu_5 = new QMenu(menubar);
        menu_5->setObjectName("menu_5");
        menu_5->setEnabled(false);
        menu_6 = new QMenu(menubar);
        menu_6->setObjectName("menu_6");
        menu_6->setEnabled(false);
        menu_7 = new QMenu(menubar);
        menu_7->setObjectName("menu_7");
        menu_7->setEnabled(false);
        menu_8 = new QMenu(menubar);
        menu_8->setObjectName("menu_8");
        menu_8->setEnabled(false);
        menu_9 = new QMenu(menubar);
        menu_9->setObjectName("menu_9");
        menu_9->setEnabled(false);
        menu_10 = new QMenu(menubar);
        menu_10->setObjectName("menu_10");
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName("toolBar");
        MainWindow->addToolBar(Qt::ToolBarArea::TopToolBarArea, toolBar);
        toolBar_2 = new QToolBar(MainWindow);
        toolBar_2->setObjectName("toolBar_2");
        MainWindow->addToolBar(Qt::ToolBarArea::LeftToolBarArea, toolBar_2);
        dockWidget_4 = new QDockWidget(MainWindow);
        dockWidget_4->setObjectName("dockWidget_4");
        dockWidget_4->setMinimumSize(QSize(200, 140));
        dockWidget_4->setMaximumSize(QSize(300, 140));
        dockWidget_4->setMouseTracking(false);
        dockWidget_4->setTabletTracking(false);
        dockWidget_4->setAcceptDrops(false);
        dockWidget_4->setAutoFillBackground(false);
        dockWidget_4->setInputMethodHints(Qt::ImhNone);
        dockWidget_4->setFloating(false);
        dockWidget_4->setFeatures(QDockWidget::DockWidgetMovable);
        dockWidgetContents_3 = new QWidget();
        dockWidgetContents_3->setObjectName("dockWidgetContents_3");
        verticalLayout = new QVBoxLayout(dockWidgetContents_3);
        verticalLayout->setObjectName("verticalLayout");
        textEdit = new QTextEdit(dockWidgetContents_3);
        textEdit->setObjectName("textEdit");
        textEdit->setEnabled(false);
        textEdit->setInputMethodHints(Qt::ImhMultiLine);
        textEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        textEdit->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        textEdit->setLineWrapMode(QTextEdit::NoWrap);
        textEdit->setReadOnly(true);

        verticalLayout->addWidget(textEdit);

        dockWidget_4->setWidget(dockWidgetContents_3);
        MainWindow->addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, dockWidget_4);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName("dockWidget");
        dockWidget->setMinimumSize(QSize(88, 107));
        dockWidget->setMaximumSize(QSize(524287, 150));
        dockWidget->setFeatures(QDockWidget::DockWidgetMovable);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName("dockWidgetContents_2");
        horizontalLayout = new QHBoxLayout(dockWidgetContents_2);
        horizontalLayout->setObjectName("horizontalLayout");
        log_textEdit = new QTextEdit(dockWidgetContents_2);
        log_textEdit->setObjectName("log_textEdit");
        log_textEdit->setReadOnly(true);

        horizontalLayout->addWidget(log_textEdit);

        dockWidget->setWidget(dockWidgetContents_2);
        MainWindow->addDockWidget(Qt::DockWidgetArea::BottomDockWidgetArea, dockWidget);
        dockWidget_2 = new QDockWidget(MainWindow);
        dockWidget_2->setObjectName("dockWidget_2");
        dockWidget_2->setFeatures(QDockWidget::DockWidgetMovable);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName("dockWidgetContents");
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName("verticalLayout_2");
        textBrowser = new QTextBrowser(dockWidgetContents);
        textBrowser->setObjectName("textBrowser");
        textBrowser->setEnabled(false);
        textBrowser->setMinimumSize(QSize(0, 80));
        textBrowser->setMaximumSize(QSize(16777215, 100));
        textBrowser->setMouseTracking(true);
        textBrowser->setAutoFillBackground(false);
        textBrowser->setAcceptRichText(true);

        verticalLayout_2->addWidget(textBrowser);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName("tabWidget");
        tab = new QWidget();
        tab->setObjectName("tab");
        verticalLayout_8 = new QVBoxLayout(tab);
        verticalLayout_8->setObjectName("verticalLayout_8");
        widget_4 = new QWidget(tab);
        widget_4->setObjectName("widget_4");
        horizontalLayout_6 = new QHBoxLayout(widget_4);
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        label = new QLabel(widget_4);
        label->setObjectName("label");
        label->setMinimumSize(QSize(0, 30));
        label->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_6->addWidget(label);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        main_view = new QPushButton(widget_4);
        main_view->setObjectName("main_view");
        main_view->setEnabled(false);
        main_view->setMinimumSize(QSize(35, 35));
        main_view->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/new/prefix1/resource/main_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        main_view->setIcon(icon);

        gridLayout->addWidget(main_view, 0, 0, 1, 1);

        back_view = new QPushButton(widget_4);
        back_view->setObjectName("back_view");
        back_view->setEnabled(false);
        back_view->setMinimumSize(QSize(35, 35));
        back_view->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/new/prefix1/resource/back_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        back_view->setIcon(icon1);

        gridLayout->addWidget(back_view, 0, 1, 1, 1);

        left_view = new QPushButton(widget_4);
        left_view->setObjectName("left_view");
        left_view->setEnabled(false);
        left_view->setMinimumSize(QSize(35, 35));
        left_view->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/new/prefix1/resource/left_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        left_view->setIcon(icon2);

        gridLayout->addWidget(left_view, 1, 0, 1, 1);

        right_view = new QPushButton(widget_4);
        right_view->setObjectName("right_view");
        right_view->setEnabled(false);
        right_view->setMinimumSize(QSize(35, 35));
        right_view->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/new/prefix1/resource/right_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        right_view->setIcon(icon3);

        gridLayout->addWidget(right_view, 1, 1, 1, 1);

        top_view = new QPushButton(widget_4);
        top_view->setObjectName("top_view");
        top_view->setEnabled(false);
        top_view->setMinimumSize(QSize(35, 35));
        top_view->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/new/prefix1/resource/top_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        top_view->setIcon(icon4);

        gridLayout->addWidget(top_view, 2, 0, 1, 1);

        upward_view = new QPushButton(widget_4);
        upward_view->setObjectName("upward_view");
        upward_view->setEnabled(false);
        upward_view->setMinimumSize(QSize(35, 35));
        upward_view->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/new/prefix1/resource/up_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        upward_view->setIcon(icon5);

        gridLayout->addWidget(upward_view, 2, 1, 1, 1);


        horizontalLayout_6->addLayout(gridLayout);


        verticalLayout_8->addWidget(widget_4);

        line = new QFrame(tab);
        line->setObjectName("line");
        line->setFrameShape(QFrame::Shape::HLine);
        line->setFrameShadow(QFrame::Shadow::Sunken);

        verticalLayout_8->addWidget(line);

        widget_3 = new QWidget(tab);
        widget_3->setObjectName("widget_3");
        horizontalLayout_5 = new QHBoxLayout(widget_3);
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        label_2 = new QLabel(widget_3);
        label_2->setObjectName("label_2");
        label_2->setMaximumSize(QSize(16777215, 30));

        horizontalLayout_5->addWidget(label_2);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName("verticalLayout_5");
        comboBox = new QComboBox(widget_3);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName("comboBox");
        comboBox->setEnabled(false);
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(comboBox->sizePolicy().hasHeightForWidth());
        comboBox->setSizePolicy(sizePolicy);
        comboBox->setMaximumSize(QSize(16777215, 22));

        verticalLayout_5->addWidget(comboBox);

        comboBox_2 = new QComboBox(widget_3);
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->setObjectName("comboBox_2");
        comboBox_2->setEnabled(false);
        comboBox_2->setMaximumSize(QSize(16777215, 22));

        verticalLayout_5->addWidget(comboBox_2);

        render_begin = new QPushButton(widget_3);
        render_begin->setObjectName("render_begin");
        render_begin->setEnabled(false);
        render_begin->setMaximumSize(QSize(16777215, 22));

        verticalLayout_5->addWidget(render_begin);


        horizontalLayout_5->addLayout(verticalLayout_5);


        verticalLayout_8->addWidget(widget_3);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName("tab_2");
        verticalLayout_6 = new QVBoxLayout(tab_2);
        verticalLayout_6->setObjectName("verticalLayout_6");
        widget_2 = new QWidget(tab_2);
        widget_2->setObjectName("widget_2");
        horizontalLayout_4 = new QHBoxLayout(widget_2);
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        label_3 = new QLabel(widget_2);
        label_3->setObjectName("label_3");

        horizontalLayout_4->addWidget(label_3);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName("verticalLayout_7");
        chazhi = new QPushButton(widget_2);
        chazhi->setObjectName("chazhi");
        chazhi->setEnabled(false);

        verticalLayout_7->addWidget(chazhi);

        chazhi1 = new QPushButton(widget_2);
        chazhi1->setObjectName("chazhi1");
        chazhi1->setEnabled(false);

        verticalLayout_7->addWidget(chazhi1);


        horizontalLayout_4->addLayout(verticalLayout_7);


        verticalLayout_6->addWidget(widget_2);

        line_2 = new QFrame(tab_2);
        line_2->setObjectName("line_2");
        line_2->setFrameShape(QFrame::Shape::HLine);
        line_2->setFrameShadow(QFrame::Shadow::Sunken);

        verticalLayout_6->addWidget(line_2);

        widget = new QWidget(tab_2);
        widget->setObjectName("widget");
        horizontalLayout_3 = new QHBoxLayout(widget);
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        label_4 = new QLabel(widget);
        label_4->setObjectName("label_4");

        horizontalLayout_3->addWidget(label_4);

        TaLanSanJiao = new QPushButton(widget);
        TaLanSanJiao->setObjectName("TaLanSanJiao");
        TaLanSanJiao->setEnabled(false);

        horizontalLayout_3->addWidget(TaLanSanJiao);


        verticalLayout_6->addWidget(widget);

        tabWidget->addTab(tab_2, QString());

        verticalLayout_2->addWidget(tabWidget);

        dockWidget_2->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, dockWidget_2);

        menubar->addAction(menu->menuAction());
        menubar->addAction(menu_2->menuAction());
        menubar->addAction(menu_3->menuAction());
        menubar->addAction(menu_4->menuAction());
        menubar->addAction(menu_5->menuAction());
        menubar->addAction(menu_6->menuAction());
        menubar->addAction(menu_7->menuAction());
        menubar->addAction(menu_8->menuAction());
        menubar->addAction(menu_9->menuAction());
        menubar->addAction(menu_10->menuAction());
        menu->addAction(open_action);
        menu->addAction(save_action);
        menu->addAction(menu_output->menuAction());
        menu->addSeparator();
        menu->addAction(close_action);
        menu->addAction(exit_action);
        menu_output->addAction(actionSTL);
        menu_output->addAction(actionOBJ);
        menu_output->addAction(actionTXT);
        menu_output->addAction(actionPLY);
        menu_output->addAction(actionPCD);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "PCL_PROCESSOR", nullptr));
        open_action->setText(QCoreApplication::translate("MainWindow", "\345\212\240\350\275\275\347\202\271\344\272\221\357\274\210open\357\274\211", nullptr));
#if QT_CONFIG(tooltip)
        open_action->setToolTip(QCoreApplication::translate("MainWindow", "\346\211\223\345\274\200\347\202\271\344\272\221\346\226\207\344\273\266", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        open_action->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+O", nullptr));
#endif // QT_CONFIG(shortcut)
        save_action->setText(QCoreApplication::translate("MainWindow", "\344\277\235\345\255\230\357\274\210save\357\274\211", nullptr));
#if QT_CONFIG(tooltip)
        save_action->setToolTip(QCoreApplication::translate("MainWindow", "\344\277\235\345\255\230\347\202\271\344\272\221\346\226\207\344\273\266", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        save_action->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+S", nullptr));
#endif // QT_CONFIG(shortcut)
        close_action->setText(QCoreApplication::translate("MainWindow", "\345\205\263\351\227\255\346\211\200\346\234\211\357\274\210close\357\274\211", nullptr));
#if QT_CONFIG(tooltip)
        close_action->setToolTip(QCoreApplication::translate("MainWindow", "\345\205\263\351\227\255\346\211\200\346\234\211\346\211\223\345\274\200\347\232\204\347\202\271\344\272\221\346\226\207\344\273\266", nullptr));
#endif // QT_CONFIG(tooltip)
        exit_action->setText(QCoreApplication::translate("MainWindow", "\351\200\200\345\207\272\347\250\213\345\272\217\357\274\210exit\357\274\211", nullptr));
        actionPCD->setText(QCoreApplication::translate("MainWindow", "PCD", nullptr));
        actionPLY->setText(QCoreApplication::translate("MainWindow", "PLY", nullptr));
        actionTXT->setText(QCoreApplication::translate("MainWindow", "TXT", nullptr));
        actionOBJ->setText(QCoreApplication::translate("MainWindow", "OBJ", nullptr));
        actionSTL->setText(QCoreApplication::translate("MainWindow", "STL", nullptr));
        menu->setTitle(QCoreApplication::translate("MainWindow", "\346\226\207\344\273\266", nullptr));
        menu_output->setTitle(QCoreApplication::translate("MainWindow", "\345\217\246\345\255\230\344\270\272(output)", nullptr));
        menu_2->setTitle(QCoreApplication::translate("MainWindow", "\347\252\227\345\217\243", nullptr));
        menu_3->setTitle(QCoreApplication::translate("MainWindow", "\345\267\245\345\205\267", nullptr));
        menu_4->setTitle(QCoreApplication::translate("MainWindow", "\347\246\273\347\276\244\347\202\271\347\247\273\351\231\244", nullptr));
        menu_5->setTitle(QCoreApplication::translate("MainWindow", "\346\273\244\346\263\242", nullptr));
        menu_6->setTitle(QCoreApplication::translate("MainWindow", "\351\205\215\345\207\206", nullptr));
        menu_7->setTitle(QCoreApplication::translate("MainWindow", "\345\205\263\351\224\256\347\202\271", nullptr));
        menu_8->setTitle(QCoreApplication::translate("MainWindow", "\350\241\250\351\235\242\351\207\215\345\273\272", nullptr));
        menu_9->setTitle(QCoreApplication::translate("MainWindow", "\345\277\253\346\215\267\351\224\256", nullptr));
        menu_10->setTitle(QCoreApplication::translate("MainWindow", "\345\270\256\345\212\251", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        toolBar_2->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar_2", nullptr));
        dockWidget_4->setWindowTitle(QCoreApplication::translate("MainWindow", "\345\212\240\350\275\275\350\277\207\347\232\204\346\226\207\344\273\266\357\274\232", nullptr));
        dockWidget->setWindowTitle(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206\345\256\236\346\227\266\346\227\245\345\277\227\357\274\232", nullptr));
        dockWidget_2->setWindowTitle(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\346\226\207\344\273\266\344\270\255\345\214\205\345\220\253\347\232\204\347\202\271\346\225\260\344\270\272\357\274\232", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "\350\247\206\345\233\276\350\275\254\346\215\242\357\274\232", nullptr));
        main_view->setText(QString());
        back_view->setText(QString());
        left_view->setText(QString());
        right_view->setText(QString());
        top_view->setText(QString());
        upward_view->setText(QString());
        label_2->setText(QCoreApplication::translate("MainWindow", "\351\253\230\347\250\213\346\270\262\346\237\223\357\274\232", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("MainWindow", "\350\275\264\345\220\221\357\274\210\345\275\223\345\211\215\344\270\272\346\227\240\357\274\211", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("MainWindow", "X\350\275\264", nullptr));
        comboBox->setItemText(2, QCoreApplication::translate("MainWindow", "Y\350\275\264", nullptr));
        comboBox->setItemText(3, QCoreApplication::translate("MainWindow", "Z\350\275\264", nullptr));

        comboBox_2->setItemText(0, QCoreApplication::translate("MainWindow", "\345\275\251\350\211\262\346\270\220\345\217\230\357\274\210\351\273\230\350\256\244\357\274\211", nullptr));
        comboBox_2->setItemText(1, QCoreApplication::translate("MainWindow", "\350\223\235\350\211\262\346\270\220\345\217\230", nullptr));
        comboBox_2->setItemText(2, QCoreApplication::translate("MainWindow", "\347\272\242\350\211\262\346\270\220\345\217\230", nullptr));
        comboBox_2->setItemText(3, QCoreApplication::translate("MainWindow", "\351\273\204\350\211\262\346\270\220\345\217\230", nullptr));

        render_begin->setText(QCoreApplication::translate("MainWindow", "\346\270\262\346\237\223", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\217\257\350\247\206\345\214\226\346\250\241\345\235\227", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "\346\217\222\345\200\274\346\223\215\344\275\234\357\274\232", nullptr));
        chazhi->setText(QCoreApplication::translate("MainWindow", "\346\234\200\350\277\221\351\202\273\346\217\222\345\200\274", nullptr));
        chazhi1->setText(QCoreApplication::translate("MainWindow", "\350\267\235\347\246\273\345\217\215\346\257\224\346\217\222\345\200\274", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "\347\275\221\346\240\274\345\214\226\346\223\215\344\275\234\357\274\232", nullptr));
        TaLanSanJiao->setText(QCoreApplication::translate("MainWindow", "\344\270\211\350\247\222\347\275\221\346\240\274\345\214\226", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206\346\250\241\345\235\227", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
