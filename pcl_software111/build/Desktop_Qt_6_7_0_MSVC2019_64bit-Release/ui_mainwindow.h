/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <CustomTreeView.h>
#include <QVTKOpenGLNativeWidget.h>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
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
    QAction *if_axis_appear;
    QAction *interface_larger;
    QAction *reset_view;
    QAction *interface_smaller;
    QAction *bb_theme;
    QAction *action1;
    QAction *action3;
    QAction *action4;
    QAction *Triangular_meshing;
    QAction *interpolation;
    QAction *actionmain_view;
    QAction *actionback_view;
    QAction *actionleft_view;
    QAction *actionright_view;
    QAction *actiontop_view;
    QAction *actionup_view;
    QAction *visualization_action;
    QAction *processing_action;
    QAction *tree_clear;
    QAction *Mouse_select;
    QAction *divide;
    QAction *tanlan;
    QAction *bosong1;
    QAction *actionStatisticalOutlierRemove;
    QAction *VoxelGrid;
    QAction *RadiusOutlinerRemoval;
    QAction *StatisticalOutlierRemoval1;
    QAction *actionX;
    QAction *actiony;
    QAction *actionZ;
    QAction *ProjectInliers;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QWidget *widget_5;
    QHBoxLayout *horizontalLayout_7;
    CustomTreeView *treeView;
    QVTKOpenGLNativeWidget *guiwidget;
    QMenuBar *menubar;
    QMenu *menu;
    QMenu *menu_output;
    QMenu *menu_2;
    QMenu *menu_4;
    QMenu *menu_5;
    QMenu *menu_3;
    QMenu *menu_7;
    QMenu *menu_8;
    QMenu *menu_9;
    QMenu *menu_10;
    QMenu *menu_11;
    QMenu *menu_12;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QToolBar *toolBar_2;
    QDockWidget *dockWidget_3;
    QWidget *dockWidgetContents_3;
    QVBoxLayout *verticalLayout_3;
    QWidget *widget_6;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_5;
    QLabel *L_yuzhi1;
    QDoubleSpinBox *L_yuzhi;
    QLabel *L_search1;
    QSpinBox *L_search;
    QPushButton *delete_away;
    QPushButton *pushButton;
    QWidget *widget_9;
    QHBoxLayout *horizontalLayout_2;
    QFormLayout *formLayout;
    QLabel *label_12;
    QSpinBox *diedai;
    QLabel *label_13;
    QDoubleSpinBox *songchi;
    QPushButton *pushButton_2;
    QFormLayout *formLayout_2;
    QLabel *label_10;
    QDoubleSpinBox *G_searchradius;
    QLabel *label_11;
    QSpinBox *G_standard;
    QPushButton *Gauss_Button;
    QFrame *line_3;
    QWidget *widget_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QVBoxLayout *verticalLayout_7;
    QPushButton *chazhi;
    QPushButton *chazhi1;
    QWidget *widget;
    QVBoxLayout *verticalLayout_6;
    QFrame *line_2;
    QLabel *label_4;
    QWidget *widget_7;
    QGridLayout *gridLayout_2;
    QDoubleSpinBox *fluent_coefficient;
    QDoubleSpinBox *searchradius;
    QLabel *label_9;
    QComboBox *comboBox_3;
    QLabel *label_8;
    QLabel *label_7;
    QLabel *label_6;
    QSpinBox *setMaximumNearestNeighbors;
    QPushButton *TaLanSanJiao;
    QPushButton *bosong;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents_2;
    QHBoxLayout *horizontalLayout;
    QTextEdit *log_textEdit;
    QDockWidget *dockWidget_2;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
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

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 924);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/new/prefix1/resource/point_cloud_icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        open_action = new QAction(MainWindow);
        open_action->setObjectName("open_action");
        open_action->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/new/prefix1/resource/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        open_action->setIcon(icon1);
        open_action->setMenuRole(QAction::NoRole);
        save_action = new QAction(MainWindow);
        save_action->setObjectName("save_action");
        save_action->setEnabled(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/new/prefix1/resource/save.png"), QSize(), QIcon::Normal, QIcon::Off);
        save_action->setIcon(icon2);
        save_action->setMenuRole(QAction::NoRole);
        close_action = new QAction(MainWindow);
        close_action->setObjectName("close_action");
        close_action->setEnabled(true);
        close_action->setMenuRole(QAction::NoRole);
        exit_action = new QAction(MainWindow);
        exit_action->setObjectName("exit_action");
        exit_action->setEnabled(true);
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
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/new/prefix1/resource/Save_As.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSTL->setIcon(icon3);
        actionSTL->setMenuRole(QAction::NoRole);
        if_axis_appear = new QAction(MainWindow);
        if_axis_appear->setObjectName("if_axis_appear");
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/new/prefix1/resource/axis.png"), QSize(), QIcon::Normal, QIcon::Off);
        if_axis_appear->setIcon(icon4);
        if_axis_appear->setMenuRole(QAction::NoRole);
        interface_larger = new QAction(MainWindow);
        interface_larger->setObjectName("interface_larger");
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/new/prefix1/resource/bigger.png"), QSize(), QIcon::Normal, QIcon::Off);
        interface_larger->setIcon(icon5);
        interface_larger->setMenuRole(QAction::NoRole);
        reset_view = new QAction(MainWindow);
        reset_view->setObjectName("reset_view");
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/new/prefix1/resource/eye.png"), QSize(), QIcon::Normal, QIcon::Off);
        reset_view->setIcon(icon6);
        reset_view->setMenuRole(QAction::NoRole);
        interface_smaller = new QAction(MainWindow);
        interface_smaller->setObjectName("interface_smaller");
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/new/prefix1/resource/smaller.png"), QSize(), QIcon::Normal, QIcon::Off);
        interface_smaller->setIcon(icon7);
        interface_smaller->setMenuRole(QAction::NoRole);
        bb_theme = new QAction(MainWindow);
        bb_theme->setObjectName("bb_theme");
        bb_theme->setMenuRole(QAction::NoRole);
        action1 = new QAction(MainWindow);
        action1->setObjectName("action1");
        action3 = new QAction(MainWindow);
        action3->setObjectName("action3");
        action4 = new QAction(MainWindow);
        action4->setObjectName("action4");
        Triangular_meshing = new QAction(MainWindow);
        Triangular_meshing->setObjectName("Triangular_meshing");
        Triangular_meshing->setEnabled(false);
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/new/prefix1/resource/meshing.png"), QSize(), QIcon::Normal, QIcon::Off);
        Triangular_meshing->setIcon(icon8);
        Triangular_meshing->setMenuRole(QAction::NoRole);
        interpolation = new QAction(MainWindow);
        interpolation->setObjectName("interpolation");
        interpolation->setEnabled(false);
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/new/prefix1/resource/interpolation.png"), QSize(), QIcon::Normal, QIcon::Off);
        interpolation->setIcon(icon9);
        interpolation->setMenuRole(QAction::NoRole);
        actionmain_view = new QAction(MainWindow);
        actionmain_view->setObjectName("actionmain_view");
        actionmain_view->setEnabled(false);
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/new/prefix1/resource/main_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionmain_view->setIcon(icon10);
        actionmain_view->setMenuRole(QAction::NoRole);
        actionback_view = new QAction(MainWindow);
        actionback_view->setObjectName("actionback_view");
        actionback_view->setEnabled(false);
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/new/prefix1/resource/back_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionback_view->setIcon(icon11);
        actionback_view->setMenuRole(QAction::NoRole);
        actionleft_view = new QAction(MainWindow);
        actionleft_view->setObjectName("actionleft_view");
        actionleft_view->setEnabled(false);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/new/prefix1/resource/left_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionleft_view->setIcon(icon12);
        actionleft_view->setMenuRole(QAction::NoRole);
        actionright_view = new QAction(MainWindow);
        actionright_view->setObjectName("actionright_view");
        actionright_view->setEnabled(false);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/new/prefix1/resource/right_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionright_view->setIcon(icon13);
        actionright_view->setMenuRole(QAction::NoRole);
        actiontop_view = new QAction(MainWindow);
        actiontop_view->setObjectName("actiontop_view");
        actiontop_view->setEnabled(false);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/new/prefix1/resource/top_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actiontop_view->setIcon(icon14);
        actiontop_view->setMenuRole(QAction::NoRole);
        actionup_view = new QAction(MainWindow);
        actionup_view->setObjectName("actionup_view");
        actionup_view->setEnabled(false);
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/new/prefix1/resource/up_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionup_view->setIcon(icon15);
        actionup_view->setMenuRole(QAction::NoRole);
        visualization_action = new QAction(MainWindow);
        visualization_action->setObjectName("visualization_action");
        visualization_action->setMenuRole(QAction::NoRole);
        processing_action = new QAction(MainWindow);
        processing_action->setObjectName("processing_action");
        processing_action->setMenuRole(QAction::NoRole);
        tree_clear = new QAction(MainWindow);
        tree_clear->setObjectName("tree_clear");
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/new/prefix1/resource/tree_clear.png"), QSize(), QIcon::Normal, QIcon::Off);
        tree_clear->setIcon(icon16);
        tree_clear->setMenuRole(QAction::NoRole);
        Mouse_select = new QAction(MainWindow);
        Mouse_select->setObjectName("Mouse_select");
        Mouse_select->setEnabled(false);
        Mouse_select->setMenuRole(QAction::NoRole);
        divide = new QAction(MainWindow);
        divide->setObjectName("divide");
        divide->setEnabled(false);
        divide->setMenuRole(QAction::NoRole);
        tanlan = new QAction(MainWindow);
        tanlan->setObjectName("tanlan");
        tanlan->setEnabled(false);
        tanlan->setMenuRole(QAction::NoRole);
        bosong1 = new QAction(MainWindow);
        bosong1->setObjectName("bosong1");
        bosong1->setEnabled(false);
        bosong1->setMenuRole(QAction::NoRole);
        actionStatisticalOutlierRemove = new QAction(MainWindow);
        actionStatisticalOutlierRemove->setObjectName("actionStatisticalOutlierRemove");
        actionStatisticalOutlierRemove->setEnabled(false);
        actionStatisticalOutlierRemove->setMenuRole(QAction::NoRole);
        VoxelGrid = new QAction(MainWindow);
        VoxelGrid->setObjectName("VoxelGrid");
        VoxelGrid->setEnabled(false);
        VoxelGrid->setMenuRole(QAction::NoRole);
        RadiusOutlinerRemoval = new QAction(MainWindow);
        RadiusOutlinerRemoval->setObjectName("RadiusOutlinerRemoval");
        RadiusOutlinerRemoval->setEnabled(false);
        RadiusOutlinerRemoval->setMenuRole(QAction::NoRole);
        StatisticalOutlierRemoval1 = new QAction(MainWindow);
        StatisticalOutlierRemoval1->setObjectName("StatisticalOutlierRemoval1");
        StatisticalOutlierRemoval1->setEnabled(false);
        StatisticalOutlierRemoval1->setMenuRole(QAction::NoRole);
        actionX = new QAction(MainWindow);
        actionX->setObjectName("actionX");
        actionX->setMenuRole(QAction::NoRole);
        actiony = new QAction(MainWindow);
        actiony->setObjectName("actiony");
        actiony->setMenuRole(QAction::NoRole);
        actionZ = new QAction(MainWindow);
        actionZ->setObjectName("actionZ");
        actionZ->setMenuRole(QAction::NoRole);
        ProjectInliers = new QAction(MainWindow);
        ProjectInliers->setObjectName("ProjectInliers");
        ProjectInliers->setEnabled(false);
        ProjectInliers->setMenuRole(QAction::NoRole);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName("verticalLayout");
        widget_5 = new QWidget(centralwidget);
        widget_5->setObjectName("widget_5");
        widget_5->setFocusPolicy(Qt::NoFocus);
        widget_5->setStyleSheet(QString::fromUtf8("background-color: black;"));
        horizontalLayout_7 = new QHBoxLayout(widget_5);
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        treeView = new CustomTreeView(widget_5);
        treeView->setObjectName("treeView");
        treeView->setMaximumSize(QSize(450, 16777215));
        treeView->setMouseTracking(false);
        treeView->setAutoFillBackground(false);
        treeView->setStyleSheet(QString::fromUtf8("color: white;"));
        treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);

        horizontalLayout_7->addWidget(treeView);

        guiwidget = new QVTKOpenGLNativeWidget(widget_5);
        guiwidget->setObjectName("guiwidget");
        guiwidget->setMouseTracking(false);
        guiwidget->setFocusPolicy(Qt::NoFocus);
        guiwidget->setAutoFillBackground(false);
        guiwidget->setStyleSheet(QString::fromUtf8("background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 #000000, stop:1 #00008B);"));

        horizontalLayout_7->addWidget(guiwidget);


        verticalLayout->addWidget(widget_5);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 17));
        menu = new QMenu(menubar);
        menu->setObjectName("menu");
        menu_output = new QMenu(menu);
        menu_output->setObjectName("menu_output");
        menu_output->setEnabled(false);
        menu_2 = new QMenu(menubar);
        menu_2->setObjectName("menu_2");
        menu_2->setEnabled(true);
        menu_4 = new QMenu(menubar);
        menu_4->setObjectName("menu_4");
        menu_4->setEnabled(true);
        menu_5 = new QMenu(menubar);
        menu_5->setObjectName("menu_5");
        menu_5->setEnabled(true);
        menu_3 = new QMenu(menu_5);
        menu_3->setObjectName("menu_3");
        menu_3->setEnabled(false);
        menu_7 = new QMenu(menubar);
        menu_7->setObjectName("menu_7");
        menu_7->setEnabled(true);
        menu_8 = new QMenu(menubar);
        menu_8->setObjectName("menu_8");
        menu_8->setEnabled(true);
        menu_9 = new QMenu(menubar);
        menu_9->setObjectName("menu_9");
        menu_9->setEnabled(true);
        menu_10 = new QMenu(menubar);
        menu_10->setObjectName("menu_10");
        menu_11 = new QMenu(menubar);
        menu_11->setObjectName("menu_11");
        menu_12 = new QMenu(menubar);
        menu_12->setObjectName("menu_12");
        menu_12->setEnabled(true);
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
        dockWidget_3 = new QDockWidget(MainWindow);
        dockWidget_3->setObjectName("dockWidget_3");
        dockWidget_3->setMinimumSize(QSize(500, 496));
        dockWidget_3->setMaximumSize(QSize(500, 800));
        dockWidget_3->setMouseTracking(false);
        dockWidget_3->setTabletTracking(false);
        dockWidget_3->setAcceptDrops(false);
        dockWidget_3->setAutoFillBackground(false);
        dockWidget_3->setInputMethodHints(Qt::ImhNone);
        dockWidget_3->setFloating(true);
        dockWidget_3->setFeatures(QDockWidget::DockWidgetClosable|QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable);
        dockWidget_3->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents_3 = new QWidget();
        dockWidgetContents_3->setObjectName("dockWidgetContents_3");
        verticalLayout_3 = new QVBoxLayout(dockWidgetContents_3);
        verticalLayout_3->setObjectName("verticalLayout_3");
        widget_6 = new QWidget(dockWidgetContents_3);
        widget_6->setObjectName("widget_6");
        verticalLayout_8 = new QVBoxLayout(widget_6);
        verticalLayout_8->setObjectName("verticalLayout_8");
        label_5 = new QLabel(widget_6);
        label_5->setObjectName("label_5");

        verticalLayout_8->addWidget(label_5);

        L_yuzhi1 = new QLabel(widget_6);
        L_yuzhi1->setObjectName("L_yuzhi1");

        verticalLayout_8->addWidget(L_yuzhi1);

        L_yuzhi = new QDoubleSpinBox(widget_6);
        L_yuzhi->setObjectName("L_yuzhi");
        L_yuzhi->setMaximum(5.000000000000000);
        L_yuzhi->setSingleStep(0.250000000000000);
        L_yuzhi->setValue(1.000000000000000);

        verticalLayout_8->addWidget(L_yuzhi);

        L_search1 = new QLabel(widget_6);
        L_search1->setObjectName("L_search1");

        verticalLayout_8->addWidget(L_search1);

        L_search = new QSpinBox(widget_6);
        L_search->setObjectName("L_search");
        L_search->setMaximum(10000);
        L_search->setSingleStep(1);
        L_search->setValue(50);

        verticalLayout_8->addWidget(L_search);

        delete_away = new QPushButton(widget_6);
        delete_away->setObjectName("delete_away");
        delete_away->setEnabled(false);

        verticalLayout_8->addWidget(delete_away);

        pushButton = new QPushButton(widget_6);
        pushButton->setObjectName("pushButton");
        pushButton->setEnabled(false);

        verticalLayout_8->addWidget(pushButton);

        widget_9 = new QWidget(widget_6);
        widget_9->setObjectName("widget_9");
        horizontalLayout_2 = new QHBoxLayout(widget_9);
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        formLayout = new QFormLayout();
        formLayout->setObjectName("formLayout");
        label_12 = new QLabel(widget_9);
        label_12->setObjectName("label_12");

        formLayout->setWidget(0, QFormLayout::LabelRole, label_12);

        diedai = new QSpinBox(widget_9);
        diedai->setObjectName("diedai");
        diedai->setMinimum(1);
        diedai->setMaximum(1000);
        diedai->setSingleStep(5);
        diedai->setValue(10);
        diedai->setDisplayIntegerBase(10);

        formLayout->setWidget(0, QFormLayout::FieldRole, diedai);

        label_13 = new QLabel(widget_9);
        label_13->setObjectName("label_13");

        formLayout->setWidget(1, QFormLayout::LabelRole, label_13);

        songchi = new QDoubleSpinBox(widget_9);
        songchi->setObjectName("songchi");
        songchi->setMinimum(0.010000000000000);
        songchi->setMaximum(1.000000000000000);
        songchi->setValue(0.500000000000000);

        formLayout->setWidget(1, QFormLayout::FieldRole, songchi);


        horizontalLayout_2->addLayout(formLayout);

        pushButton_2 = new QPushButton(widget_9);
        pushButton_2->setObjectName("pushButton_2");
        pushButton_2->setEnabled(false);

        horizontalLayout_2->addWidget(pushButton_2);

        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName("formLayout_2");
        label_10 = new QLabel(widget_9);
        label_10->setObjectName("label_10");

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_10);

        G_searchradius = new QDoubleSpinBox(widget_9);
        G_searchradius->setObjectName("G_searchradius");
        G_searchradius->setMaximum(9999.000000000000000);
        G_searchradius->setValue(10.000000000000000);

        formLayout_2->setWidget(0, QFormLayout::FieldRole, G_searchradius);

        label_11 = new QLabel(widget_9);
        label_11->setObjectName("label_11");

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_11);

        G_standard = new QSpinBox(widget_9);
        G_standard->setObjectName("G_standard");
        G_standard->setMaximum(10000);
        G_standard->setSingleStep(10);
        G_standard->setValue(6);

        formLayout_2->setWidget(1, QFormLayout::FieldRole, G_standard);


        horizontalLayout_2->addLayout(formLayout_2);

        Gauss_Button = new QPushButton(widget_9);
        Gauss_Button->setObjectName("Gauss_Button");
        Gauss_Button->setEnabled(false);

        horizontalLayout_2->addWidget(Gauss_Button);


        verticalLayout_8->addWidget(widget_9);


        verticalLayout_3->addWidget(widget_6);

        line_3 = new QFrame(dockWidgetContents_3);
        line_3->setObjectName("line_3");
        line_3->setFrameShape(QFrame::Shape::HLine);
        line_3->setFrameShadow(QFrame::Shadow::Sunken);

        verticalLayout_3->addWidget(line_3);

        widget_2 = new QWidget(dockWidgetContents_3);
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


        verticalLayout_3->addWidget(widget_2);

        widget = new QWidget(dockWidgetContents_3);
        widget->setObjectName("widget");
        verticalLayout_6 = new QVBoxLayout(widget);
        verticalLayout_6->setObjectName("verticalLayout_6");
        line_2 = new QFrame(widget);
        line_2->setObjectName("line_2");
        line_2->setFrameShape(QFrame::Shape::HLine);
        line_2->setFrameShadow(QFrame::Shadow::Sunken);

        verticalLayout_6->addWidget(line_2);

        label_4 = new QLabel(widget);
        label_4->setObjectName("label_4");
        label_4->setMaximumSize(QSize(16777215, 50));

        verticalLayout_6->addWidget(label_4);

        widget_7 = new QWidget(widget);
        widget_7->setObjectName("widget_7");
        gridLayout_2 = new QGridLayout(widget_7);
        gridLayout_2->setObjectName("gridLayout_2");
        fluent_coefficient = new QDoubleSpinBox(widget_7);
        fluent_coefficient->setObjectName("fluent_coefficient");
        fluent_coefficient->setSingleStep(0.500000000000000);
        fluent_coefficient->setValue(3.500000000000000);

        gridLayout_2->addWidget(fluent_coefficient, 0, 4, 2, 2);

        searchradius = new QDoubleSpinBox(widget_7);
        searchradius->setObjectName("searchradius");
        searchradius->setMaximum(9999.000000000000000);
        searchradius->setValue(10.000000000000000);

        gridLayout_2->addWidget(searchradius, 0, 1, 2, 2);

        label_9 = new QLabel(widget_7);
        label_9->setObjectName("label_9");

        gridLayout_2->addWidget(label_9, 2, 3, 1, 2);

        comboBox_3 = new QComboBox(widget_7);
        comboBox_3->addItem(QString());
        comboBox_3->addItem(QString());
        comboBox_3->setObjectName("comboBox_3");

        gridLayout_2->addWidget(comboBox_3, 2, 5, 1, 1);

        label_8 = new QLabel(widget_7);
        label_8->setObjectName("label_8");

        gridLayout_2->addWidget(label_8, 2, 0, 1, 2);

        label_7 = new QLabel(widget_7);
        label_7->setObjectName("label_7");

        gridLayout_2->addWidget(label_7, 0, 3, 2, 1);

        label_6 = new QLabel(widget_7);
        label_6->setObjectName("label_6");

        gridLayout_2->addWidget(label_6, 0, 0, 2, 1);

        setMaximumNearestNeighbors = new QSpinBox(widget_7);
        setMaximumNearestNeighbors->setObjectName("setMaximumNearestNeighbors");
        setMaximumNearestNeighbors->setMaximum(10000);
        setMaximumNearestNeighbors->setSingleStep(10);
        setMaximumNearestNeighbors->setValue(1500);

        gridLayout_2->addWidget(setMaximumNearestNeighbors, 2, 2, 1, 1);


        verticalLayout_6->addWidget(widget_7);

        TaLanSanJiao = new QPushButton(widget);
        TaLanSanJiao->setObjectName("TaLanSanJiao");
        TaLanSanJiao->setEnabled(false);

        verticalLayout_6->addWidget(TaLanSanJiao);

        bosong = new QPushButton(widget);
        bosong->setObjectName("bosong");
        bosong->setEnabled(false);

        verticalLayout_6->addWidget(bosong);


        verticalLayout_3->addWidget(widget);

        dockWidget_3->setWidget(dockWidgetContents_3);
        MainWindow->addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, dockWidget_3);
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
        dockWidget_2->setAcceptDrops(false);
        dockWidget_2->setFloating(true);
        dockWidget_2->setFeatures(QDockWidget::DockWidgetClosable|QDockWidget::DockWidgetMovable);
        dockWidget_2->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName("dockWidgetContents");
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName("verticalLayout_2");
        widget_4 = new QWidget(dockWidgetContents);
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
        main_view->setIcon(icon10);

        gridLayout->addWidget(main_view, 0, 0, 1, 1);

        back_view = new QPushButton(widget_4);
        back_view->setObjectName("back_view");
        back_view->setEnabled(false);
        back_view->setMinimumSize(QSize(35, 35));
        back_view->setMaximumSize(QSize(16777215, 16777215));
        back_view->setIcon(icon11);

        gridLayout->addWidget(back_view, 0, 1, 1, 1);

        left_view = new QPushButton(widget_4);
        left_view->setObjectName("left_view");
        left_view->setEnabled(false);
        left_view->setMinimumSize(QSize(35, 35));
        left_view->setMaximumSize(QSize(16777215, 16777215));
        left_view->setIcon(icon12);

        gridLayout->addWidget(left_view, 1, 0, 1, 1);

        right_view = new QPushButton(widget_4);
        right_view->setObjectName("right_view");
        right_view->setEnabled(false);
        right_view->setMinimumSize(QSize(35, 35));
        right_view->setMaximumSize(QSize(16777215, 16777215));
        right_view->setIcon(icon13);

        gridLayout->addWidget(right_view, 1, 1, 1, 1);

        top_view = new QPushButton(widget_4);
        top_view->setObjectName("top_view");
        top_view->setEnabled(false);
        top_view->setMinimumSize(QSize(35, 35));
        top_view->setMaximumSize(QSize(16777215, 16777215));
        top_view->setIcon(icon14);

        gridLayout->addWidget(top_view, 2, 0, 1, 1);

        upward_view = new QPushButton(widget_4);
        upward_view->setObjectName("upward_view");
        upward_view->setEnabled(false);
        upward_view->setMinimumSize(QSize(35, 35));
        upward_view->setMaximumSize(QSize(16777215, 16777215));
        upward_view->setIcon(icon15);

        gridLayout->addWidget(upward_view, 2, 1, 1, 1);


        horizontalLayout_6->addLayout(gridLayout);


        verticalLayout_2->addWidget(widget_4);

        line = new QFrame(dockWidgetContents);
        line->setObjectName("line");
        line->setFrameShape(QFrame::Shape::HLine);
        line->setFrameShadow(QFrame::Shadow::Sunken);

        verticalLayout_2->addWidget(line);

        widget_3 = new QWidget(dockWidgetContents);
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


        verticalLayout_2->addWidget(widget_3);

        dockWidget_2->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, dockWidget_2);

        menubar->addAction(menu->menuAction());
        menubar->addAction(menu_2->menuAction());
        menubar->addAction(menu_4->menuAction());
        menubar->addAction(menu_5->menuAction());
        menubar->addAction(menu_7->menuAction());
        menubar->addAction(menu_12->menuAction());
        menubar->addAction(menu_8->menuAction());
        menubar->addAction(menu_9->menuAction());
        menubar->addAction(menu_10->menuAction());
        menubar->addAction(menu_11->menuAction());
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
        menu_2->addAction(visualization_action);
        menu_2->addAction(processing_action);
        menu_4->addAction(actionStatisticalOutlierRemove);
        menu_5->addAction(VoxelGrid);
        menu_5->addAction(menu_3->menuAction());
        menu_5->addAction(RadiusOutlinerRemoval);
        menu_5->addAction(StatisticalOutlierRemoval1);
        menu_5->addAction(ProjectInliers);
        menu_3->addAction(actionX);
        menu_3->addAction(actiony);
        menu_3->addAction(actionZ);
        menu_8->addAction(tanlan);
        menu_8->addAction(bosong1);
        menu_11->addAction(bb_theme);
        menu_11->addAction(action1);
        menu_11->addAction(action3);
        menu_11->addAction(action4);
        menu_12->addAction(Mouse_select);
        menu_12->addAction(divide);
        toolBar->addAction(open_action);
        toolBar->addAction(save_action);
        toolBar->addAction(actionSTL);
        toolBar->addSeparator();
        toolBar->addAction(Triangular_meshing);
        toolBar->addAction(interpolation);
        toolBar->addAction(tree_clear);
        toolBar_2->addAction(if_axis_appear);
        toolBar_2->addAction(reset_view);
        toolBar_2->addAction(interface_larger);
        toolBar_2->addAction(interface_smaller);
        toolBar_2->addAction(actionmain_view);
        toolBar_2->addAction(actionback_view);
        toolBar_2->addAction(actionleft_view);
        toolBar_2->addAction(actionright_view);
        toolBar_2->addAction(actiontop_view);
        toolBar_2->addAction(actionup_view);

        retranslateUi(MainWindow);

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
        if_axis_appear->setText(QCoreApplication::translate("MainWindow", "\345\235\220\346\240\207\350\275\264\346\230\276\347\244\272", nullptr));
        interface_larger->setText(QCoreApplication::translate("MainWindow", "\347\225\214\351\235\242\346\224\276\345\244\247", nullptr));
        reset_view->setText(QCoreApplication::translate("MainWindow", "\350\247\206\350\247\222\345\244\215\344\275\215", nullptr));
        interface_smaller->setText(QCoreApplication::translate("MainWindow", "\347\225\214\351\235\242\347\274\251\345\260\217", nullptr));
        bb_theme->setText(QCoreApplication::translate("MainWindow", "\347\231\275\350\223\235\351\243\216\346\240\274", nullptr));
        action1->setText(QCoreApplication::translate("MainWindow", "\345\216\237\351\243\216\346\240\274", nullptr));
        action3->setText(QCoreApplication::translate("MainWindow", "\351\273\221\350\211\262\351\243\216\346\240\274", nullptr));
        action4->setText(QCoreApplication::translate("MainWindow", "\347\231\275\351\273\221\351\243\216\346\240\274", nullptr));
        Triangular_meshing->setText(QCoreApplication::translate("MainWindow", "\344\270\211\350\247\222\347\275\221\346\240\274\345\214\226", nullptr));
        interpolation->setText(QCoreApplication::translate("MainWindow", "\346\234\200\350\277\221\351\202\273\346\217\222\345\200\274", nullptr));
        actionmain_view->setText(QCoreApplication::translate("MainWindow", "main_view", nullptr));
        actionback_view->setText(QCoreApplication::translate("MainWindow", "back_view", nullptr));
        actionleft_view->setText(QCoreApplication::translate("MainWindow", "left_view", nullptr));
        actionright_view->setText(QCoreApplication::translate("MainWindow", "right_view", nullptr));
        actiontop_view->setText(QCoreApplication::translate("MainWindow", "top_view", nullptr));
        actionup_view->setText(QCoreApplication::translate("MainWindow", "up_view", nullptr));
        visualization_action->setText(QCoreApplication::translate("MainWindow", "\345\217\257\350\247\206\345\214\226\346\223\215\344\275\234\347\252\227\345\217\243", nullptr));
        processing_action->setText(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206\347\252\227\345\217\243", nullptr));
        tree_clear->setText(QCoreApplication::translate("MainWindow", "\346\270\205\347\251\272\346\240\221\347\212\266\345\233\276", nullptr));
        Mouse_select->setText(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\241\206\351\200\211", nullptr));
#if QT_CONFIG(tooltip)
        Mouse_select->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\241\206\351\200\211\345\207\272\350\246\201\344\277\235\347\225\231\347\232\204\347\202\271\344\272\221\351\203\250\345\210\206", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        Mouse_select->setShortcut(QCoreApplication::translate("MainWindow", "F", nullptr));
#endif // QT_CONFIG(shortcut)
        divide->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\345\210\206\345\211\262", nullptr));
        tanlan->setText(QCoreApplication::translate("MainWindow", "\350\264\252\345\251\252\344\270\211\350\247\222\345\214\226", nullptr));
        bosong1->setText(QCoreApplication::translate("MainWindow", "\346\263\212\346\235\276\351\207\215\345\273\272", nullptr));
        actionStatisticalOutlierRemove->setText(QCoreApplication::translate("MainWindow", "StatisticalOutlierRemove", nullptr));
        VoxelGrid->setText(QCoreApplication::translate("MainWindow", "\344\275\223\347\264\240\346\273\244\346\263\242\345\231\250", nullptr));
#if QT_CONFIG(tooltip)
        VoxelGrid->setToolTip(QCoreApplication::translate("MainWindow", "VoxelGrid", nullptr));
#endif // QT_CONFIG(tooltip)
        RadiusOutlinerRemoval->setText(QCoreApplication::translate("MainWindow", "\345\215\212\345\276\204\347\246\273\347\276\244\347\202\271\345\216\273\351\231\244\346\273\244\346\263\242\345\231\250", nullptr));
#if QT_CONFIG(tooltip)
        RadiusOutlinerRemoval->setToolTip(QCoreApplication::translate("MainWindow", "RadiusOutlinerRemoval", nullptr));
#endif // QT_CONFIG(tooltip)
        StatisticalOutlierRemoval1->setText(QCoreApplication::translate("MainWindow", "\347\273\237\350\256\241\347\246\273\347\276\244\347\202\271\345\216\273\351\231\244\346\273\244\346\263\242\345\231\250", nullptr));
#if QT_CONFIG(tooltip)
        StatisticalOutlierRemoval1->setToolTip(QCoreApplication::translate("MainWindow", "StatisticalOutlierRemoval", nullptr));
#endif // QT_CONFIG(tooltip)
        actionX->setText(QCoreApplication::translate("MainWindow", "X\350\275\264", nullptr));
        actiony->setText(QCoreApplication::translate("MainWindow", "Y\350\275\264", nullptr));
        actionZ->setText(QCoreApplication::translate("MainWindow", "Z\350\275\264", nullptr));
        ProjectInliers->setText(QCoreApplication::translate("MainWindow", "\346\212\225\345\275\261\345\217\202\346\225\260\345\214\226\346\250\241\345\236\213\346\273\244\346\263\242\345\231\250", nullptr));
#if QT_CONFIG(tooltip)
        ProjectInliers->setToolTip(QCoreApplication::translate("MainWindow", "ProjectInliers", nullptr));
#endif // QT_CONFIG(tooltip)
        menu->setTitle(QCoreApplication::translate("MainWindow", "\346\226\207\344\273\266", nullptr));
        menu_output->setTitle(QCoreApplication::translate("MainWindow", "\345\217\246\345\255\230\344\270\272(output)", nullptr));
        menu_2->setTitle(QCoreApplication::translate("MainWindow", "\347\252\227\345\217\243", nullptr));
        menu_4->setTitle(QCoreApplication::translate("MainWindow", "\347\246\273\347\276\244\347\202\271\347\247\273\351\231\244", nullptr));
        menu_5->setTitle(QCoreApplication::translate("MainWindow", "\346\273\244\346\263\242", nullptr));
        menu_3->setTitle(QCoreApplication::translate("MainWindow", "\347\233\264\351\200\232\346\273\244\346\263\242\345\231\250", nullptr));
        menu_7->setTitle(QCoreApplication::translate("MainWindow", "\345\205\263\351\224\256\347\202\271", nullptr));
        menu_8->setTitle(QCoreApplication::translate("MainWindow", "\350\241\250\351\235\242\351\207\215\345\273\272", nullptr));
        menu_9->setTitle(QCoreApplication::translate("MainWindow", "\345\277\253\346\215\267\351\224\256", nullptr));
        menu_10->setTitle(QCoreApplication::translate("MainWindow", "\345\270\256\345\212\251", nullptr));
        menu_11->setTitle(QCoreApplication::translate("MainWindow", "\344\270\273\351\242\230", nullptr));
        menu_12->setTitle(QCoreApplication::translate("MainWindow", "\345\210\206\345\211\262", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        toolBar_2->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar_2", nullptr));
        dockWidget_3->setWindowTitle(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206\346\223\215\344\275\234\347\225\214\351\235\242", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "\351\242\204\345\244\204\347\220\206\346\223\215\344\275\234\357\274\232", nullptr));
        L_yuzhi1->setText(QCoreApplication::translate("MainWindow", "\345\210\244\346\226\255\351\230\210\345\200\274\357\274\232", nullptr));
        L_search1->setText(QCoreApplication::translate("MainWindow", "\350\200\203\350\231\221\344\270\264\350\277\221\347\202\271\344\270\252\346\225\260\357\274\232", nullptr));
        delete_away->setText(QCoreApplication::translate("MainWindow", "\347\246\273\347\276\244\347\202\271\345\216\273\351\231\244", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "\344\270\213\351\207\207\346\240\267", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "\345\271\263\346\273\221\350\277\255\344\273\243\346\254\241\346\225\260\357\274\232", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "\346\235\276\345\274\233\345\233\240\345\255\220\357\274\232", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "\346\213\211\346\231\256\346\213\211\346\226\257\345\271\263\346\273\221\345\244\204\347\220\206", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "\346\220\234\347\264\242\345\215\212\345\276\204\357\274\232", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "\346\240\207\345\207\206\345\267\256\357\274\232", nullptr));
        Gauss_Button->setText(QCoreApplication::translate("MainWindow", "\351\253\230\346\226\257\345\271\263\346\273\221\345\244\204\347\220\206", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "\346\217\222\345\200\274\346\223\215\344\275\234\357\274\232", nullptr));
        chazhi->setText(QCoreApplication::translate("MainWindow", "\346\234\200\350\277\221\351\202\273\346\217\222\345\200\274", nullptr));
        chazhi1->setText(QCoreApplication::translate("MainWindow", "\350\267\235\347\246\273\345\217\215\346\257\224\346\217\222\345\200\274", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "\347\275\221\346\240\274\345\214\226\346\223\215\344\275\234\357\274\232", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "\350\200\203\350\231\221\347\202\271\347\232\204\346\263\225\347\272\277\344\270\200\350\207\264\346\200\247:", nullptr));
        comboBox_3->setItemText(0, QCoreApplication::translate("MainWindow", "\345\220\246", nullptr));
        comboBox_3->setItemText(1, QCoreApplication::translate("MainWindow", "\346\230\257", nullptr));

        label_8->setText(QCoreApplication::translate("MainWindow", "\346\234\200\345\244\247\351\202\273\345\261\205\347\202\271\346\225\260\357\274\232", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "\345\205\211\346\273\221\345\257\206\345\272\246\345\233\240\345\255\220\357\274\232", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "\346\220\234\347\264\242\345\215\212\345\276\204\357\274\232", nullptr));
        TaLanSanJiao->setText(QCoreApplication::translate("MainWindow", "\350\277\233\350\241\214\350\264\252\345\251\252\344\270\211\350\247\222\345\214\226", nullptr));
        bosong->setText(QCoreApplication::translate("MainWindow", "\346\263\212\346\235\276\351\207\215\345\273\272", nullptr));
        dockWidget->setWindowTitle(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206\345\256\236\346\227\266\346\227\245\345\277\227\357\274\232", nullptr));
        dockWidget_2->setWindowTitle(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\345\217\257\350\247\206\345\214\226\346\223\215\344\275\234\347\225\214\351\235\242", nullptr));
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
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
