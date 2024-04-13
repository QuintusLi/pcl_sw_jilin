#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//设置输出日志颜色的常量
#define RED     QColor(192,0,0)
#define GREEN   QColor(0,192,0)
#define YELLOW  QColor(192,192,0)
#define BLACK   QColor(0,0,0)
#define GRAY    QColor(128,128,128)

#include <QMainWindow>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>//各种格式的点的头文件
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkElevationFilter.h>//高程渲染所用的头文件
#include <vtkCamera.h>//切换视图相关头文件
#include <vtkBoundingBox.h>//切换视图相关头文件
#include <vtkRendererCollection.h>//切换视图相关头文件
#include <vtkMath.h>//切换视图相关头文件
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

//输出日志用
typedef struct{
    QtMsgType type;
    QString txt;
}LogElement;
//输出日志用
extern QQueue<LogElement>log_txt;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow();





private slots:

    void on_main_view_clicked();

    void on_back_view_clicked();

    void on_left_view_clicked();

    void on_right_view_clicked();

    void on_upward_view_clicked();

    void on_top_view_clicked();

    void on_Timer0Timeout();//输出日志用

    void on_render_begin_clicked();

    void on_render_begin_pressed();

    void setButtonsEnabled(bool enabled); // 禁用新添加的函数声明


private:
    Ui::MainWindow *ui;
    QTimer *timer0;

    boost::shared_ptr< pcl::visualization::PCLVisualizer > view;//加载点云的共享指针
    void updateCameraView(double focalPoint[3], double position[3], double viewUp[3]);//切换视图用
    bool buttonsEnabled;//存储按钮状态


};
#endif // MAINWINDOW_H
