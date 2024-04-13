#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>//各种格式的点的头文件
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <vtkGenericOpenGLRenderWindow.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();



private:
    Ui::MainWindow *ui;

    boost::shared_ptr< pcl::visualization::PCLVisualizer > view;//加载点云的共享指针


};
#endif // MAINWINDOW_H
