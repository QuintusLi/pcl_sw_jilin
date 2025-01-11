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
#include <QCheckBox>
#include <QStandardItemModel>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/mouse_event.h> //鼠标事件
#include <pcl/visualization/keyboard_event.h>//键盘事件
#include <pcl/filters/project_inliers.h>
#include "CustomTreeView.h"

extern int count1;
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

    void mouseEventOccurred(vtkObject* caller, unsigned long eventId, void* callData);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon;//用于储存鼠标框选的框选点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped1;//用于存储点云在多边形内部的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped2;//用于存储点云在多边形内部的点
    pcl::PointXYZ curP, lastP;
    bool flag = false;
    bool isPickingMode = false;
    unsigned int line_id = 0;
    boost::shared_ptr< pcl::visualization::PCLVisualizer > view;//加载点云的共享指针

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

    void on_if_axis_appear_triggered();

    void on_reset_view_triggered();

    void on_interface_larger_triggered();

    void on_interface_smaller_triggered();

    void on_bb_theme_triggered();

    void on_action1_triggered();

    void on_action3_triggered();

    void on_action4_triggered();

    void on_actionmain_view_triggered();

    void on_actionback_view_triggered();

    void on_actionleft_view_triggered();

    void on_actionright_view_triggered();

    void on_actiontop_view_triggered();

    void on_actionup_view_triggered();

    void on_visualization_action_triggered();

    void on_processing_action_triggered();

    void on_tree_clear_triggered();

    void checkState();

    void getScreentPos(double* displayPos, double* world,void* viewer_void);

    int inOrNot1(int poly_sides, double *poly_X, double *poly_Y, double x, double y);

    void projectInliers(void*);

    void on_Mouse_select_triggered();

    void on_divide_triggered();

    void on_tanlan_triggered();

    void on_bosong1_triggered();

    void on_actionStatisticalOutlierRemove_triggered();

    void on_actionPLY_triggered();

    void on_actionPCD_triggered();

    void on_actionTXT_triggered();

    void on_actionOBJ_triggered();

    void on_close_action_triggered();

    void on_StatisticalOutlierRemoval1_triggered();

    void on_VoxelGrid_triggered();

    void on_RadiusOutlinerRemoval_triggered();

    void on_actionX_triggered();

    void on_actiony_triggered();

    void on_actionZ_triggered();

    void on_ProjectInliers_triggered();

private:
    Ui::MainWindow *ui;
    CustomTreeView *customTreeView;
    QTimer *timer0;

    void updateCameraView(double focalPoint[3], double position[3], double viewUp[3]);//切换视图用
    bool buttonsEnabled;//存储按钮状态

    vtkSmartPointer<vtkOrientationMarkerWidget> MarkerWidget;
    QStandardItemModel *model;
    QStandardItem *fileItem;
    std::string cloud_id;                                              //点云的id
    pcl::PointCloud<pcl::PointXYZ>::Ptr allcloud;
    QMap<QString, pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloudMap;  // 存储点云名称和对应指针
    QMap<QString, bool> pointCloudVisibilityMap;  // 存储点云名称和其显示状态
    QMap<QString, pcl::PolygonMesh> polygonMeshMap;  // 存储生成的 PolygonMesh 对象
    QMap<QString, bool> polygonMeshVisibilityMap;  // 存储 PolygonMesh 的可见性状态
    int checkcount = 0; //为复选框的勾选数量计数
    std::pair<QString, pcl::PointCloud<pcl::PointXYZ>::Ptr> getCurrentlyDisplayedPointCloudFromView(); //查找当前正在显示的点云
    //判断两点云是否相同（此函数为辅助用）
    bool arePointCloudsEqual(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) const {
        if (cloud1->size()!= cloud2->size()) {
            return false;
        }
        for (size_t i = 0; i < cloud1->size(); ++i) {
            if (cloud1->points[i].x!= cloud2->points[i].x || cloud1->points[i].y!= cloud2->points[i].y || cloud1->points[i].z!= cloud2->points[i].z) {
                return false;
            }
        }
        return true;
    }



};
#endif // MAINWINDOW_H
