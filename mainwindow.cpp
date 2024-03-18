#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QFileDialog>
MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    view.reset(new pcl::visualization::PCLVisualizer(renderer,renderWindow,"viewer",false));
    view->setupInteractor(ui->guiwidget->interactor(),ui->guiwidget->renderWindow());
    ui->guiwidget->setRenderWindow(view->getRenderWindow());

    //打开点云文件的匿名信号槽
    connect(ui->open_action,&QAction::triggered,this,[=]{
        //读取pcd文件并显示
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//创建点云指针
        QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.pcd)");
        if(fileName == "") return;
        pcl::io::loadPCDFile(fileName.toStdString(),*cloud);
        view->addPointCloud(cloud,"cloud");
        view->resetCamera();    //视角
        ui->guiwidget->renderWindow()->Render();
    });

    //pcd格式转ply格式
    connect(ui->actionPLY,&QAction::triggered,this,[=]{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.pcd)");
        if(fileName == "") return;
        pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud);
        //pcl::PLYWriter writer;
        //writer.write(QString("/home/li/pcd/"+fileName.section('/',-1,-1).section('.',0,0)+".ply").toStdString(), *cloud);
        pcl::io::savePLYFile("test.ply", *cloud);

    });




}

MainWindow::~MainWindow()
{
    delete ui;
}




