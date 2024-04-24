#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QFileDialog>
#include <QMessageBox>
#include<QDebug>
#include<QTimer>
#include<QQueue>
#include<vtkColorTransferFunction.h>
#include <vtkScalarBarActor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <QTreeView>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QStandardItem>
#include <QDir>
#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <QWheelEvent>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkVersion.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkSmoothPolyDataFilter.h>



QQueue<LogElement> log_txt;
int count1=0;
MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent), ui(new Ui::MainWindow), buttonsEnabled(false)
{
    ui->setupUi(this);
    customTreeView = qobject_cast<CustomTreeView*>(ui->treeView);
    ui->dockWidget_2->hide();
    ui->dockWidget_3->hide();
    timer0=new QTimer(this);
    timer0->setInterval(300);
    connect(timer0, &QTimer::timeout,this,&MainWindow::on_Timer0Timeout);
    timer0->start();

    qDebug()<<"正在初始化可视化窗口......";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>);
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    view.reset(new pcl::visualization::PCLVisualizer(renderer,renderWindow,"viewer",false));
    view->setupInteractor(ui->guiwidget->interactor(),ui->guiwidget->renderWindow());
    view->addCoordinateSystem(0.1);

    pcl::PointXYZ position_x,position_y,position_z;//设置坐标轴标签
    position_x.x = 0.11f;
    position_x.y = 0.0f;
    position_x.z = 0.0f;
    position_y.x = 0.0f;
    position_y.y = 0.11f;
    position_y.z = 0.0f;
    position_z.x = 0.0f;
    position_z.y = 0.0f;
    position_z.z = 0.11f;
    view->addText3D("X",position_x,0.01, 1.0, 1.0, 1.0,"x_label");
    view->addText3D("Y",position_y,0.01, 1.0, 1.0, 1.0,"y_label");
    view->addText3D("Z",position_z,0.01, 1.0, 1.0, 1.0,"z_label");
    ui->guiwidget->setRenderWindow(view->getRenderWindow());
    qDebug()<<"可视化窗口初始化成功！可以进行操作！";
    QStandardItemModel *model = new QStandardItemModel(this);
    //设置表头隐藏
    ui->treeView->setHeaderHidden(true);
    //设置model
    ui->treeView->setModel(model);
    //设置展开
    ui->treeView->expandAll();


    // 显示坐标系的vtk组件
    vtkSmartPointer<vtkAxesActor> axes_actor = vtkSmartPointer<vtkAxesActor>::New();
    axes_actor->SetPosition(0, 0, 0);
    axes_actor->SetTotalLength(2, 2, 2);
    axes_actor->SetShaftType(0);
    axes_actor->SetCylinderRadius(0.03);
    axes_actor->SetAxisLabels(1);
    axes_actor->SetTipType(0);
    axes_actor->SetXAxisLabelText("x");
    axes_actor->GetXAxisShaftProperty()->SetColor(1,1,1);
    axes_actor->GetYAxisTipProperty()->SetColor(1,1,1);

    // 控制坐标系，使之随视角共同变化
    MarkerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    MarkerWidget->SetOrientationMarker(axes_actor);
    MarkerWidget->SetInteractor(ui->guiwidget->interactor());
    MarkerWidget->SetViewport(0.0, 0.0, 0.2, 0.2);
    MarkerWidget->SetEnabled(1);
    MarkerWidget->SetOutlineColor(1,0,0);








    //打开点云文件的匿名信号槽
    connect(ui->open_action,&QAction::triggered,this,[cloudptr,this]{
        // 首先尝试从视图中移除之前的点云
        view->removeAllPointClouds();
        ui->guiwidget->update();//更新ui->guiwidget上的显示
        qDebug()<<"正在打开文件"<<"......";
        QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "", tr ("Point cloud data (*.txt *.pcd *.ply)"));
        PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

        if (filename.isEmpty ()){
            qDebug()<<"打开失败，文件为空！";
            return;}

        QString filePath = QFileInfo(filename).absoluteFilePath(); // 获取文件的完整路径

        QStandardItemModel *model = qobject_cast<QStandardItemModel*>(ui->treeView->model());

        if (!model) {
            model = new QStandardItemModel(this);
            ui->treeView->setModel(model);
        }


        QStandardItem *rootItem = model->invisibleRootItem();

        QDir dir;
        QString currentPath = filePath;
        QStringList pathParts = filePath.split(QDir::separator());

        // 遍历文件的完整路径，构建目录树
        for (int i = 0; i < pathParts.size(); ++i) {
            QString part = pathParts.at(i);
            bool found = false;
            QStandardItem *parentItem = nullptr;

            // 在当前级别查找对应的目录项
            for (int j = 0; j < rootItem->rowCount(); ++j) {
                if (rootItem->child(j)->text() == part) {
                    parentItem = rootItem->child(j);
                    found = true;
                    break;
                }
            }

            // 如果目录项不存在，则创建它
            if (!found) {
                parentItem = new QStandardItem(part);
                parentItem->setIcon(QIcon(":/new/prefix1/resource/file.png")); // 设置文件图标
                parentItem->setData(true, Qt::UserRole); // 存储是目录的信息
                rootItem->appendRow(parentItem);
            }

            // 移动到下一级目录项作为父项
            rootItem = parentItem;
        }

        // 在文件路径的最后添加文件项
        QStandardItem *fileItem = new QStandardItem(QFileInfo(filePath).fileName());
        fileItem->setIcon(QIcon(":/new/prefix1/resource/cloud.png")); // 设置文件图标
        fileItem->setData(false, Qt::UserRole); // 存储不是目录的信息
        rootItem->appendRow(fileItem);
        ui->treeView->expandAll(); // 展开所有项
        ui->guiwidget->update();//确保更新ui->guiwidget上的显示


        int return_status=1;
        if (filename.endsWith (".pcd", Qt::CaseInsensitive))
            {
            count1=count1+1;//为打开过的文件计数
            return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloudptr);
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";

        }
        else if(filename.endsWith (".ply", Qt::CaseInsensitive)){
            count1=count1+1;//为打开过的文件计数
            return_status = pcl::io::loadPLYFile(filename.toStdString (), *cloudptr);
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";
        }
        else if (filename.endsWith (".txt", Qt::CaseInsensitive))
        {
            cloudptr->clear();
            count1=count1+1;//为打开过的文件计数
            std::ifstream file(filename.toStdString().c_str());
            std::string line;//定义行
            pcl::PointXYZ point;
            while (getline(file, line)) //按行读取文件
            {
                // 找到第一个逗号的位置
                size_t commaPos = line.find(",");
                if (commaPos != std::string::npos)
                {
                    // 跳过逗号及其之前的部分
                    line = line.substr(commaPos + 1);
                }
                std::string::iterator it;
                for (it = line.begin(); it < line.end(); it++)
                {
                    if (*it == ',')//判断是否有，
                    {
                        line.erase(it);//删除，
                        line.insert(it, ' ');//插入空格
                        it--;
                    }
                }
                std::stringstream ss(line);//点云赋值
                ss >> point.x;
                ss >> point.y;
                ss >> point.z;
                cloudptr->push_back(point);
            }
            return_status = 0;
            file.close();
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";
        }
        else
            {
            cloudptr->clear();
            count1=count1+1;//为打开过的文件计数
            std::ifstream file(filename.toStdString().c_str());
            std::string line;//定义行
            pcl::PointXYZ point;
            while (getline(file, line)) //按行读取文件
            {
                // 找到第一个逗号的位置
                size_t commaPos = line.find(",");
                if (commaPos != std::string::npos)
                {
                    // 跳过逗号及其之前的部分
                    line = line.substr(commaPos + 1);
                }
                std::string::iterator it;
                for (it = line.begin(); it < line.end(); it++)
                {
                    if (*it == ',')//判断是否有，
                    {
                        line.erase(it);//删除，
                        line.insert(it, ' ');//插入空格
                        it--;
                    }
                }
                std::stringstream ss(line);//点云赋值
                ss >> point.x;
                ss >> point.y;
                ss >> point.z;
                cloudptr->push_back(point);
            }
            return_status = 0;
            file.close();
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";
            }

        if (return_status != 0)
        {
            PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
            qDebug()<<"文件:"<<filename.toStdString ().c_str ()<<"打开失败，请重新操作！";
            return;
        }
        // 加载点云成功后解除按钮的禁用状态
        setButtonsEnabled(true);

        //计算点的数量
        // 获取点云中点的数量并将其转换为字符串
        int point_num = cloudptr->size();
        QString point_num_str = QString::number(point_num);
        QString a = QString::number(count1);//转换类型，方便显示
        QStandardItem *pointcount = new QStandardItem("文件包含点的数量为："+point_num_str+"个");
        fileItem->appendRow(pointcount);

        // 将点云数据转换为VTK的点数据
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        for (size_t i = 0; i < cloudptr->size(); ++i) {
            pcl::PointXYZ p = cloudptr->at(i);
            points->InsertNextPoint(p.x, p.y, p.z);
        }
        std::string cloud_id = "cloud_" + std::to_string(count1);
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr,cloud_id);
        view->resetCamera();    //视角
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();//更新ui->guiwidget上的显示

        //计算质心
        // 创建一个Eigen::Vector4f对象来存储质心
        Eigen::Vector4f centroid1;
        bool centroid_computed = pcl::compute3DCentroid(*cloudptr, centroid1);

        // 检查质心是否计算成功
        if (!centroid_computed)
        {
            PCL_ERROR("3D centroid computation failed.\n");
            return;
        }
        QString centroid1_x = QString::number(centroid1[0]);
        QString centroid1_y = QString::number(centroid1[1]);
        QString centroid1_z = QString::number(centroid1[2]);
        QStandardItem *centroid = new QStandardItem("点云的质心坐标为：["+centroid1_x+","+centroid1_y+","+centroid1_z+"]");
        fileItem->appendRow(centroid);

        //计算点云的表面积
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(modifiedCloud);
        n.setInputCloud(modifiedCloud);
        n.setSearchMethod(tree);
        n.setKSearch(10);
        n.compute(*normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;
        gp3.setSearchRadius(10);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(1500);
        gp3.setMaximumSurfaceAngle(M_PI / 4);
        gp3.setMinimumAngle(M_PI / 36);
        gp3.setMaximumAngle(2 * M_PI / 3);
        gp3.setNormalConsistency(false); // Reconstruct
        gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);
    // 计算网格面积
    double area = 0.0;
    for (const auto& polygon : triangles.polygons)
    {
        for (size_t i = 2; i < polygon.vertices.size(); ++i)
        {
            const pcl::PointXYZ& v0 = modifiedCloud->points[polygon.vertices[0]];
            const pcl::PointXYZ& v1 = modifiedCloud->points[polygon.vertices[i - 1]];
            const pcl::PointXYZ& v2 = modifiedCloud->points[polygon.vertices[i]];
            Eigen::Vector3f edge1 = v1.getVector3fMap() - v0.getVector3fMap();
            Eigen::Vector3f edge2 = v2.getVector3fMap() - v0.getVector3fMap();

            double triangleArea = 0.5 * std::abs(edge1.cross(edge2).norm());
            area += triangleArea;
        }
    }

        QString b = QString::number(area);
        QStandardItem *surface_area = new QStandardItem("点云的表面积为："+b);
        fileItem->appendRow(surface_area);

        double volume1 = 0.0;
        Eigen::Vector3f origin(centroid1[0],centroid1[1],centroid1[2]); // 可以选择其他点作为四面体的顶点

        for (const auto& face : triangles.polygons) {
            if (face.vertices.size() != 3) continue; // 确保是三角形

            const pcl::PointXYZ& p1 = modifiedCloud->points[face.vertices[0]];
            const pcl::PointXYZ& p2 = modifiedCloud->points[face.vertices[1]];
            const pcl::PointXYZ& p3 = modifiedCloud->points[face.vertices[2]];
            Eigen::Vector3f v01 = p1.getVector3fMap() - origin;
            Eigen::Vector3f v02 = p2.getVector3fMap() - origin;
            Eigen::Vector3f v03 = p3.getVector3fMap() - origin;
            double result=std::abs(v01.dot(v02.cross(v03))) / 6.0;

            volume1 += result;
        }
        QString c = QString::number(volume1);
        QStandardItem *volume = new QStandardItem("点云的体积为："+c);
        fileItem->appendRow(volume);

        //计算点云密度
        double density1 = point_num / volume1;
        QString d = QString::number(density1);
        QStandardItem *density = new QStandardItem("点云的密度为："+d);
        fileItem->appendRow(density);
        ui->treeView->expandAll(); // 展开所有项
    });

    //最近邻插值运算
    connect(ui->chazhi,&QPushButton::clicked,this,[=]{
        qDebug()<<"正在进行最近邻插值......";
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        // 创建KdTree对象进行最近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(modifiedCloud);
        // 对每个点进行最近邻搜索并插值
        for (size_t i = 0; i < modifiedCloud->points.size(); ++i) {
            std::vector<int> pointIdxNKNSearch(2); // 存储两个最近邻点的索引
            std::vector<float> pointNKNSquaredDistance(2); // 存储两个最近邻点的距离
            int num_nn = kdtree->nearestKSearch(modifiedCloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance);

            if (num_nn == 2) { // 确保找到了两个最近邻点
                // 在原始点和其最近邻点之间插入一个点
                pcl::PointXYZ newPoint;
                newPoint.x = (modifiedCloud->points[i].x + modifiedCloud->points[pointIdxNKNSearch[1]].x) / 2.0; // 使用第二个最近邻点（索引为1）
                newPoint.y = (modifiedCloud->points[i].y + modifiedCloud->points[pointIdxNKNSearch[1]].y) / 2.0;
                newPoint.z = (modifiedCloud->points[i].z + modifiedCloud->points[pointIdxNKNSearch[1]].z) / 2.0;
                cloudptr->push_back(newPoint);
                view->removeAllPointClouds();
                view->addPointCloud(cloudptr);
                ui->guiwidget->renderWindow()->Render();
                ui->guiwidget->update();
            }
        }
        qDebug()<<"成功完成最近邻插值!";
    });
    //最近邻插值运算
    connect(ui->interpolation,&QAction::triggered,this,[=]{
        qDebug()<<"正在进行最近邻插值......";
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        // 创建KdTree对象进行最近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(modifiedCloud);
        // 对每个点进行最近邻搜索并插值
        for (size_t i = 0; i < modifiedCloud->points.size(); ++i) {
            std::vector<int> pointIdxNKNSearch(2); // 存储两个最近邻点的索引
            std::vector<float> pointNKNSquaredDistance(2); // 存储两个最近邻点的距离
            int num_nn = kdtree->nearestKSearch(modifiedCloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance);

            if (num_nn == 2) { // 确保找到了两个最近邻点
                // 在原始点和其最近邻点之间插入一个点
                pcl::PointXYZ newPoint;
                newPoint.x = (modifiedCloud->points[i].x + modifiedCloud->points[pointIdxNKNSearch[1]].x) / 2.0; // 使用第二个最近邻点（索引为1）
                newPoint.y = (modifiedCloud->points[i].y + modifiedCloud->points[pointIdxNKNSearch[1]].y) / 2.0;
                newPoint.z = (modifiedCloud->points[i].z + modifiedCloud->points[pointIdxNKNSearch[1]].z) / 2.0;
                cloudptr->push_back(newPoint);
                view->removeAllPointClouds();
                view->addPointCloud(cloudptr);
                ui->guiwidget->renderWindow()->Render();
                ui->guiwidget->update();
            }
        }
        qDebug()<<"成功完成最近邻插值!";
    });

    //距离反比插值运算
    connect(ui->chazhi1,&QPushButton::clicked,this,[=]{
        qDebug()<<"正在进行距离反比插值......";
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        // 创建KdTree对象进行最近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(modifiedCloud);
    // 设置反比插值的幂次，这里使用2次幂
    const float power = 2.0f;

    // 对每个点进行搜索并插值
    for (size_t i = 0; i < modifiedCloud->points.size(); ++i) {
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        // 设置搜索的最近邻点数量，这里可能需要调整以获得更好的插值效果
        const int num_nn = 10; // 假设我们使用10个最近邻点进行插值
        int num_found = kdtree->nearestKSearch(modifiedCloud->points[i], num_nn, pointIdxNKNSearch, pointNKNSquaredDistance);

        if (num_found >= 2) { // 确保至少找到了两个最近邻点
            pcl::PointXYZ newPoint;
            float sumWeights = 0.0f;
            float weightSumX = 0.0f, weightSumY = 0.0f, weightSumZ = 0.0f;

            // 对每个最近邻点进行反比插值
            for (int j = 0; j < num_found; ++j) {
                float distance = sqrt(pointNKNSquaredDistance[j]);
                float weight = 1.0f / pow(distance, power); // 计算权重
                sumWeights += weight;

                // 累加带权重的坐标
                weightSumX += weight * modifiedCloud->points[pointIdxNKNSearch[j]].x;
                weightSumY += weight * modifiedCloud->points[pointIdxNKNSearch[j]].y;
                weightSumZ += weight * modifiedCloud->points[pointIdxNKNSearch[j]].z;
            }

            // 计算加权平均后的坐标
            newPoint.x = weightSumX / sumWeights;
            newPoint.y = weightSumY / sumWeights;
            newPoint.z = weightSumZ / sumWeights;

            // 将插值后的点添加到输出点云中
            cloudptr->push_back(newPoint);
            view->removeAllPointClouds();
            view->addPointCloud(cloudptr);
            ui->guiwidget->renderWindow()->Render();
            ui->guiwidget->update();
            }
        }
        qDebug()<<"成功完成距离反比插值!";
    });

    //保存点云
    connect(ui->save_action,&QAction::triggered,this,[=]{
        qDebug()<<"正在将您的点云文件保存......";
        int return_status;
        QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply *.stl)"));

        if (cloudptr->empty())
        {
            qDebug()<<"您未打开文件，无法进行保存操作！";
            return;
        } else {
            if (filename.isEmpty()) {
                qDebug()<<"保存失败，文件为空！";
                return;
            }
            if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
                return_status = pcl::io::savePCDFileASCII(filename.toStdString(), *cloudptr);
                qDebug()<<"已成功保存为PCD文件！";
            } else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
                return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
                qDebug()<<"已成功保存为PLY文件！";
            } else if(filename.endsWith(".stl", Qt::CaseInsensitive)) {
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
                if (modifiedCloud->empty()) {
                    qDebug()<<"保存失败，未找到用于保存的点云！";
                    return;
                }

                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(modifiedCloud);
                n.setInputCloud(modifiedCloud);
                n.setSearchMethod(tree);
                n.setKSearch(10);
                n.compute(*normals);
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
                pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
                tree2->setInputCloud(cloud_with_normals);
                pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
                pcl::PolygonMesh triangles;

                // Set the maximum distance between connected points (maximum edge length)
                gp3.setSearchRadius(10);
                gp3.setMu(2.5);
                gp3.setMaximumNearestNeighbors(1500);
                gp3.setMaximumSurfaceAngle(M_PI / 4);
                gp3.setMinimumAngle(M_PI / 36);
                gp3.setMaximumAngle(2 * M_PI / 3);
                gp3.setNormalConsistency(false); // Reconstruct
                gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
                gp3.reconstruct(triangles); // Save STL file pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
                pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
                return_status = 0;
                qDebug()<<"已成功保存为stl文件！";
            }else {
                filename.append(".stl");
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
                if (modifiedCloud->empty()) {
                    qDebug()<<"保存失败，请重新操作！";
                    return;
                }

                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(modifiedCloud);
                n.setInputCloud(modifiedCloud);
                n.setSearchMethod(tree);
                n.setKSearch(10);
                n.compute(*normals);
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
                pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
                tree2->setInputCloud(cloud_with_normals);
                pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
                pcl::PolygonMesh triangles;

                // Set the maximum distance between connected points (maximum edge length)
                gp3.setSearchRadius(10);
                gp3.setMu(2.5);
                gp3.setMaximumNearestNeighbors(1500);
                gp3.setMaximumSurfaceAngle(M_PI / 4);
                gp3.setMinimumAngle(M_PI / 36);
                gp3.setMaximumAngle(2 * M_PI / 3);
                gp3.setNormalConsistency(false); // Reconstruct
                gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
                gp3.reconstruct(triangles); // Save STL file pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
                pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
                return_status = 0;
                qDebug()<<"已成功保存为stl文件！";
            }
            if (return_status != 0) {
                PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
                qDebug()<<"保存失败，请重新操作！";
                return;
            }
        }
    });

    //转stl
    connect(ui->actionSTL, &QAction::triggered, this, [cloudptr,this]{
        qDebug()<<"正在将文件另存为stl格式......";
        QString filename = QFileDialog::getSaveFileName(this, tr("Save STL File"), "", tr("STL Files (*.stl)"));
        if (filename.isEmpty()) {
            qDebug()<<"转换失败，文件为空！";
            return;
        }
        if (filename.endsWith(".stl", Qt::CaseInsensitive)==false) {
            filename.append(".stl");
        }
            // 创建点云副本，使用智能指针管理内存
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
            if (modifiedCloud->empty()) {
                qDebug()<<"保存失败，未找到用于转换的点云！";
                return;
            }

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(modifiedCloud);
            n.setInputCloud(modifiedCloud);
            n.setSearchMethod(tree);
            n.setKSearch(10);
            n.compute(*normals);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_with_normals);
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            pcl::PolygonMesh triangles;

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius(10);
            gp3.setMu(2.5);
            gp3.setMaximumNearestNeighbors(1500);
            gp3.setMaximumSurfaceAngle(M_PI / 4);
            gp3.setMinimumAngle(M_PI / 36);
            gp3.setMaximumAngle(2 * M_PI / 3);
            gp3.setNormalConsistency(false); // Reconstruct
            gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
            gp3.reconstruct(triangles); // Save STL file pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
            pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
            qDebug()<<"成功另存为stl格式！";
            return;

    });

    //贪婪三角化并可视化
    connect(ui->TaLanSanJiao,&QPushButton::clicked,this,[=]{
        qDebug()<<"正在进行贪婪三角网格化......";
        // 创建点云副本，使用智能指针管理内存
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        if (modifiedCloud->empty()) {
            return;
        }
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(modifiedCloud);
        n.setInputCloud(modifiedCloud);
        n.setSearchMethod(tree);
        n.setKSearch(10);
        n.compute(*normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(10);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(1500);
        gp3.setMaximumSurfaceAngle(M_PI / 4);
        gp3.setMinimumAngle(M_PI / 36);
        gp3.setMaximumAngle(2 * M_PI / 3);
        gp3.setNormalConsistency(false); // Reconstruct
        gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);
        view->removeAllPointClouds();
        view->addPolygonMesh(triangles,"mytriangles");
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"贪婪三角网格化处理完成！";
    });

    //拉普拉斯平滑处理
    connect(ui->pushButton_2,&QPushButton::clicked,this,[=]{
        qDebug()<<"正在进行拉普拉斯平滑处理......";
        vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
        // 创建拉普拉斯平滑滤波器
        vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
            vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        smoothFilter->SetInputData(polydata);

        // 设置平滑迭代次数和松弛因子
        smoothFilter->SetNumberOfIterations(20); // 迭代次数，根据需要进行调整
        smoothFilter->SetRelaxationFactor(0.9); // 松弛因子，控制平滑的强度

        // 执行平滑处理
        smoothFilter->Update();
        // 获取平滑后的vtkPolyData
        vtkPolyData* smoothedPolyData = smoothFilter->GetOutput();
        // 获取vtkPolyData中的点数
        vtkIdType numPoints = smoothedPolyData->GetNumberOfPoints();
        cloudptr->clear();

        // 遍历vtkPolyData中的每个点
        for (vtkIdType i = 0; i < numPoints; ++i)
        {
            double point[3];
            smoothedPolyData->GetPoint(i, point); // 获取点的坐标

            // 创建PCL点，并设置其坐标
            pcl::PointXYZ pclPoint;
            pclPoint.x = point[0];
            pclPoint.y = point[1];
            pclPoint.z = point[2];

            // 将PCL点添加到点云中
            cloudptr->points.push_back(pclPoint);
        }

            view->removeAllPointClouds();
            view->addPointCloud(cloudptr);
            ui->guiwidget->renderWindow()->Render();
            ui->guiwidget->update();
            qDebug()<<"拉普拉斯平滑处理已完成！";

    });

    //下采样
    connect(ui->pushButton,&QPushButton::clicked,this,[=]{
        qDebug()<<"正在进行下采样......";
        // 创建点云副本，使用智能指针管理内存
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        if (modifiedCloud->empty()) {
            qDebug()<<"下采样失败！";
            return;
        }
        // 计算点云中两点之间的最小距离，用于设定合适的叶尺寸
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < modifiedCloud->size(); ++i) {
            for (size_t j = i + 1; j < modifiedCloud->size(); ++j) {
                double dist = std::sqrt(std::pow(modifiedCloud->points[i].x - modifiedCloud->points[j].x, 2) +
                                        std::pow(modifiedCloud->points[i].y - modifiedCloud->points[j].y, 2) +
                                        std::pow(modifiedCloud->points[i].z - modifiedCloud->points[j].z, 2));
                if (dist < min_distance) {
                    min_distance = dist;
                }
            }
        }
        float leaf_size = min_distance * 1.1f;
        // 创建体素网格滤波器对象
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(modifiedCloud);
        // 设置体素网格的大小（以米为单位），这将决定下采样后的点云分辨率
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        // 应用滤波器并获取下采样后的点云
        sor.filter(*cloudptr);

        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"下采样处理完成！";
    });
    //贪婪三角化并可视化（图标）
    connect(ui->Triangular_meshing,&QAction::triggered,this,[=]{
        qDebug()<<"正在进行贪婪三角网格化......";
        // 创建点云副本，使用智能指针管理内存
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        if (modifiedCloud->empty()) {
            qDebug()<<"点云不存在!";
            return;
        }
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(modifiedCloud);
        n.setInputCloud(modifiedCloud);
        n.setSearchMethod(tree);
        n.setKSearch(10);
        n.compute(*normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(10);
        gp3.setMu(3.5);
        gp3.setMaximumNearestNeighbors(1500);
        gp3.setMaximumSurfaceAngle(M_PI / 4);
        gp3.setMinimumAngle(M_PI / 36);
        gp3.setMaximumAngle(2 * M_PI / 3);
        gp3.setNormalConsistency(false); // Reconstruct
        gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);
        view->removeAllPointClouds();
        view->addPolygonMesh(triangles,"mytriangles");
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"贪婪三角网格化处理完成！";
    });




    //退出程序
    connect(ui->exit_action,&QAction::triggered,this,[=]{
        if (QMessageBox::question(this, tr("exit tip"), tr("你确定要退出吗 ?"), QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes)
        {
            qApp->quit();
        }
    });




}

MainWindow::~MainWindow()
{
    delete ui;
}

//禁用功能实现函数
void MainWindow::setButtonsEnabled(bool enabled)
{

    // 更改ui为可用
    ui->save_action->setEnabled(enabled);
    ui->exit_action->setEnabled(enabled);
    ui->main_view->setEnabled(enabled);
    ui->back_view->setEnabled(enabled);
    ui->left_view->setEnabled(enabled);
    ui->right_view->setEnabled(enabled);
    ui->upward_view->setEnabled(enabled);
    ui->top_view->setEnabled(enabled);
    ui->render_begin->setEnabled(enabled);
    ui->comboBox->setEnabled(enabled);
    ui->comboBox_2->setEnabled(enabled);
    ui->main_view->setEnabled(enabled);
    ui->back_view->setEnabled(enabled);
    ui->left_view->setEnabled(enabled);
    ui->right_view->setEnabled(enabled);
    ui->upward_view->setEnabled(enabled);
    ui->top_view->setEnabled(enabled);
    ui->render_begin->setEnabled(enabled);
    ui->menu_3->setEnabled(enabled);
    ui->menu_4->setEnabled(enabled);
    ui->menu_5->setEnabled(enabled);
    ui->menu_6->setEnabled(enabled);
    ui->menu_7->setEnabled(enabled);
    ui->menu_8->setEnabled(enabled);
    ui->menu_9->setEnabled(enabled);
    ui->chazhi->setEnabled(enabled);
    ui->chazhi1->setEnabled(enabled);
    ui->TaLanSanJiao->setEnabled(enabled);
    ui->actionSTL->setEnabled(enabled);
    ui->Triangular_meshing->setEnabled(enabled);
    ui->interpolation->setEnabled(enabled);
    ui->actionmain_view->setEnabled(enabled);
    ui->actionback_view->setEnabled(enabled);
    ui->actionleft_view->setEnabled(enabled);
    ui->actionright_view->setEnabled(enabled);
    ui->actionup_view->setEnabled(enabled);
    ui->actiontop_view->setEnabled(enabled);
    ui->pushButton->setEnabled(enabled);
    ui->pushButton_2->setEnabled(enabled);



}

//切换视图用主函数
void MainWindow::updateCameraView(double focalPoint[3], double position[3], double viewUp[3])
{
    vtkCamera* camera = ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
    camera->SetFocalPoint(focalPoint);
    camera->SetPosition(position);
    camera->SetViewUp(viewUp);
    // 刷新渲染窗口
    ui->guiwidget->renderWindow()->Render();
}

//切换主视图
void MainWindow::on_main_view_clicked()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0], center[1]-cam_distance, center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为主视图！";
}
void MainWindow::on_actionmain_view_triggered()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0], center[1]-cam_distance, center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为主视图！";
}

//切换后视图
void MainWindow::on_back_view_clicked()
{

    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[0]-bounds[1], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0], center[1]+cam_distance, center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为后视图！";
}
void MainWindow::on_actionback_view_triggered()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[0]-bounds[1], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0], center[1]+cam_distance, center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为后视图！";
}

//切换左视图
void MainWindow::on_left_view_clicked()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0]-cam_distance, center[1], center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为左视图！";
}
void MainWindow::on_actionleft_view_triggered()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0]-cam_distance, center[1], center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为左视图！";
}

//切换右视图
void MainWindow::on_right_view_clicked()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0]+cam_distance, center[1], center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为右视图！";
}
void MainWindow::on_actionright_view_triggered()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double position[3] = {center[0]+cam_distance, center[1], center[2]};
    double viewUp[3] = {0.0, 0.0, 1.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为右视图！";
}

//切换仰视图
void MainWindow::on_upward_view_clicked()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1]-bounds[0], bounds[5]-bounds[4], bounds[2]-bounds[3]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double cam_height = cam_distance * tan(vtkMath::Pi()/4.0);
    double position[3] = {center[0], center[1], center[2]-cam_height};
    double viewUp[3] = {0.0, -1.0, 0.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为仰视图！";
}
void MainWindow::on_actiontop_view_triggered()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1]-bounds[0], bounds[5]-bounds[4], bounds[2]-bounds[3]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double cam_height = cam_distance * tan(vtkMath::Pi()/4.0);
    double position[3] = {center[0], center[1], center[2]-cam_height};
    double viewUp[3] = {0.0, -1.0, 0.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为仰视图！";
}

//切换俯视图
void MainWindow::on_top_view_clicked()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double cam_height = cam_distance * tan(vtkMath::Pi()/4.0);
    double position[3] = {center[0], center[1], center[2]+cam_height};
    double viewUp[3] = {0.0, 1.0, 0.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为俯视图！";
}
void MainWindow::on_actionup_view_triggered()
{
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    double center[3];
    polydata->GetCenter(center);
    double bounds[6];
    polydata->GetBounds(bounds);
    double max_range = std::max({bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]});
    double cam_distance = max_range / tan(45.0*vtkMath::Pi()/360.0);
    double cam_height = cam_distance * tan(vtkMath::Pi()/4.0);
    double position[3] = {center[0], center[1], center[2]+cam_height};
    double viewUp[3] = {0.0, 1.0, 0.0};
    updateCameraView(center, position, viewUp);
    qDebug()<<"已切换为俯视图！";
}

//日志实时输出
void MainWindow::on_Timer0Timeout()
{
    QTextCharFormat tcf;
    while(log_txt.count()){
        LogElement log_element=log_txt.dequeue();
        switch (log_element.type) {
        case QtMsgType::QtInfoMsg:
            tcf.setForeground(Qt::blue);
            break;
        case QtMsgType::QtWarningMsg:
            tcf.setForeground(Qt::red);
            break;
        default:
            tcf.setForeground(Qt::black);
            break;
        }
        ui->log_textEdit->moveCursor(QTextCursor::End);
        ui->log_textEdit->textCursor().insertText(log_element.txt,tcf);
    }
}



//单色高程渲染
void MainWindow::on_render_begin_clicked()
{
    int index1 = ui->comboBox->currentIndex();
    int index2 = ui->comboBox_2->currentIndex();
    if(index2!=0&&index1!=0)
    {
    qDebug()<<"正在进行高程渲染......";
    // 从QVTKOpenGLWidget中获取vtkRenderWindow
    vtkRenderWindow* renderWindow = ui->guiwidget->renderWindow();
    // 获取renderer和actor
    vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
    vtkActor* actor = renderer->GetActors()->GetLastActor();
    // 创建颜色映射条
    vtkSmartPointer<vtkScalarBarActor> scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();

    // 创建VTK的高程渲染对象   
    vtkSmartPointer<vtkElevationFilter> elevationFilter = vtkSmartPointer<vtkElevationFilter>::New();
    elevationFilter->SetInputConnection(actor->GetMapper()->GetInputConnection(0, 0));
    // 创建颜色映射器
    vtkSmartPointer<vtkColorTransferFunction> colorTransferFunction = vtkSmartPointer<vtkColorTransferFunction>::New();
    // 创建VTK的mapper和actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(elevationFilter->GetOutputPort());




    //蓝色渐变渲染部分
    if(index1==1&&index2==1)//x轴
    {
        elevationFilter->SetLowPoint(actor->GetBounds()[1], 0, 0);
        elevationFilter->SetHighPoint(actor->GetBounds()[0], 0, 0);
        colorTransferFunction->AddRGBPoint(1.0, 0.0, 0.0, 1.0); // 深蓝色
        colorTransferFunction->AddRGBPoint(0.0, 0.0, 1.0, 1.0); // 浅蓝色
    }
    if(index1==2&&index2==1)//y轴
    {
        elevationFilter->SetLowPoint(0, actor->GetBounds()[3], 0);
        elevationFilter->SetHighPoint(0, actor->GetBounds()[2], 0);
        colorTransferFunction->AddRGBPoint(1.0, 0.0, 0.0, 1.0); // 深蓝色
        colorTransferFunction->AddRGBPoint(0.0, 0.0, 1.0, 1.0); // 浅蓝色
    }
    if(index1==3&&index2==1)//z轴
    {
        elevationFilter->SetLowPoint(0, 0, actor->GetBounds()[5]);
        elevationFilter->SetHighPoint(0, 0,actor->GetBounds()[4]);
        colorTransferFunction->AddRGBPoint(1.0, 0.0, 0.0, 1.0); // 深蓝色
        colorTransferFunction->AddRGBPoint(0.0, 0.0, 1.0, 1.0); // 浅蓝色
    }

    //红色渐变渲染部分
    if(index1==1&&index2==2)//x轴
    {
        elevationFilter->SetLowPoint(actor->GetBounds()[1], 0, 0);
        elevationFilter->SetHighPoint(actor->GetBounds()[0], 0, 0);
        colorTransferFunction->AddRGBPoint(1.0, 0.8, 0.0, 0.0); // 深红色
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 0.8, 0.8); // 浅红色
    }
    if(index1==2&&index2==2)//y轴
    {
        elevationFilter->SetLowPoint(0,actor->GetBounds()[3], 0);
        elevationFilter->SetHighPoint(0,actor->GetBounds()[2], 0);
        colorTransferFunction->AddRGBPoint(1.0, 0.8, 0.0, 0.0); // 深红色
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 0.8, 0.8); // 浅红色
    }
    if(index1==3&&index2==2)//z轴
    {
        elevationFilter->SetLowPoint(0, 0, actor->GetBounds()[5]);
        elevationFilter->SetHighPoint(0, 0,actor->GetBounds()[4]);
        colorTransferFunction->AddRGBPoint(1.0, 0.8, 0.0, 0.0); // 深红色
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 0.8, 0.8); // 浅红色
    }

    //黄色渐变渲染部分
    if(index1==1&&index2==3)//x轴
    {
        elevationFilter->SetLowPoint(actor->GetBounds()[1], 0, 0);
        elevationFilter->SetHighPoint(actor->GetBounds()[0], 0, 0);
        colorTransferFunction->AddRGBPoint(1.0, 0.72, 0.52, 0.04); // 深黄色
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 1.0, 0.0); // 浅黄色
    }
    if(index1==2&&index2==3)//y轴
    {
        elevationFilter->SetLowPoint(0,actor->GetBounds()[3], 0);
        elevationFilter->SetHighPoint(0,actor->GetBounds()[2], 0);
        colorTransferFunction->AddRGBPoint(1.0,0.72, 0.52, 0.04); // 深黄色
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 1.0, 0.0); // 浅黄色
    }
    if(index1==3&&index2==3)//z轴
    {
        elevationFilter->SetLowPoint(0, 0, actor->GetBounds()[5]);
        elevationFilter->SetHighPoint(0, 0,actor->GetBounds()[4]);
        colorTransferFunction->AddRGBPoint(1.0, 0.72, 0.52, 0.04); // 深黄色
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 1.0, 0.0); // 浅黄色
    }


    mapper->SetLookupTable(colorTransferFunction);
    actor->SetMapper(mapper);
    scalarBar->SetLookupTable(colorTransferFunction);
    scalarBar->SetTitle("Elevation");
    //将Actor和颜色映射条添加到渲染器中
    renderer->AddActor(actor);
    renderer->AddActor2D(scalarBar);
    renderWindow->Render();
    renderer->RemoveActor2D(scalarBar);
    qDebug()<<"渲染完成！";

    }
    else
        return;
}



//彩色高程渲染
void MainWindow::on_render_begin_pressed()
{
    int index1 = ui->comboBox->currentIndex();
    int index2 = ui->comboBox_2->currentIndex();
    if(index2==0&&index1!=0)
    {
    qDebug()<<"正在进行高程渲染......";
    // 从QVTKOpenGLWidget中获取vtkRenderWindow
    vtkRenderWindow* renderWindow = ui->guiwidget->renderWindow();
    // 获取renderer和actor
    vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
    vtkActor* actor = renderer->GetActors()->GetLastActor();
    // 创建颜色映射条
    vtkSmartPointer<vtkScalarBarActor> scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();

    // 创建VTK的高程渲染对象
    vtkSmartPointer<vtkElevationFilter> elevationFilter = vtkSmartPointer<vtkElevationFilter>::New();
    elevationFilter->SetInputConnection(actor->GetMapper()->GetInputConnection(0, 0));
    // 创建颜色映射器
    vtkSmartPointer<vtkColorTransferFunction> colorTransferFunction = vtkSmartPointer<vtkColorTransferFunction>::New();
    // 创建VTK的mapper和actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(elevationFilter->GetOutputPort());


    //彩色渐变渲染部分
    if(index1==1)//x轴
    {
        elevationFilter->SetLowPoint(actor->GetBounds()[1], 0, 0);
        elevationFilter->SetHighPoint(actor->GetBounds()[0], 0, 0);
        colorTransferFunction->AddRGBPoint(0.0, 0.0, 0.0, 1.0); // 蓝色
        colorTransferFunction->AddRGBPoint(0.33, 0.0, 1.0, 0.0); //绿色
        colorTransferFunction->AddRGBPoint(0.66, 1.0, 1.0, 0.0); //黄色
        colorTransferFunction->AddRGBPoint(1.0, 1.0, 0.0, 0.0); //红色
    }
    if(index1==2)//y轴
    {
        elevationFilter->SetLowPoint(0, actor->GetBounds()[3], 0);
        elevationFilter->SetHighPoint(0, actor->GetBounds()[2], 0);
        colorTransferFunction->AddRGBPoint(0.0, 0.0, 0.0, 1.0); // 蓝色
        colorTransferFunction->AddRGBPoint(0.33, 0.0, 1.0, 0.0); //绿色
        colorTransferFunction->AddRGBPoint(0.66, 1.0, 1.0, 0.0); //黄色
        colorTransferFunction->AddRGBPoint(1.0, 1.0, 0.0, 0.0); //红色
    }
    if(index1==3)//z轴
    {
        elevationFilter->SetLowPoint(0, 0, actor->GetBounds()[5]);
        elevationFilter->SetHighPoint(0, 0, actor->GetBounds()[4]);
        colorTransferFunction->AddRGBPoint(0.0, 0.0, 0.0, 1.0); // 蓝色
        colorTransferFunction->AddRGBPoint(0.33, 0.0, 1.0, 0.0); //绿色
        colorTransferFunction->AddRGBPoint(0.66, 1.0, 1.0, 0.0); //黄色
        colorTransferFunction->AddRGBPoint(1.0, 1.0, 0.0, 0.0); //红色
    }


    mapper->SetLookupTable(colorTransferFunction);
    actor->SetMapper(mapper);
    scalarBar->SetLookupTable(colorTransferFunction);
    scalarBar->SetTitle("Elevation");
    //将Actor和颜色映射条添加到渲染器中
    renderer->AddActor(actor);
    renderer->AddActor2D(scalarBar);
    renderWindow->Render();
    renderer->RemoveActor2D(scalarBar);
    qDebug()<<"渲染完成！";
    }
    else
        return;
}


//控制坐标轴是否显示
void MainWindow::on_if_axis_appear_triggered()
{
    static bool axisVisible = true; // 初始状态为显示坐标轴

    if (axisVisible) {
        view->removeCoordinateSystem(); // 隐藏坐标轴
        view->removeText3D("x_label");
        view->removeText3D("y_label");
        view->removeText3D("z_label");
    } else {
        view->addCoordinateSystem(0.1); // 显示坐标轴
        pcl::PointXYZ position_x,position_y,position_z;//设置坐标轴标签
        position_x.x = 0.11f;
        position_x.y = 0.0f;
        position_x.z = 0.0f;
        position_y.x = 0.0f;
        position_y.y = 0.11f;
        position_y.z = 0.0f;
        position_z.x = 0.0f;
        position_z.y = 0.0f;
        position_z.z = 0.11f;
        view->addText3D("X",position_x,0.01, 1.0, 1.0, 1.0,"x_label");
        view->addText3D("Y",position_y,0.01, 1.0, 1.0, 1.0,"y_label");
        view->addText3D("Z",position_z,0.01, 1.0, 1.0, 1.0,"z_label");
    }
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();

    axisVisible = !axisVisible; // 切换状态
}

//界面放大
void MainWindow::on_interface_larger_triggered()
{
    vtkCamera* camera = ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
    double currentFovy = camera->GetViewAngle(); // 获取当前FOV
    double newFovy = currentFovy * 0.8; // 根据需要调整放大倍数
    camera->SetViewAngle(newFovy); // 设置新的FOV
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
}
//界面缩小
void MainWindow::on_interface_smaller_triggered()
{
    vtkCamera* camera = ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
    double currentFovy = camera->GetViewAngle(); // 获取当前FOV
    double newFovy = currentFovy * 1.2; // 根据需要调整放大倍数
    camera->SetViewAngle(newFovy); // 设置新的FOV
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
}

//视角复位
void MainWindow::on_reset_view_triggered()
{
    view->resetCamera();    //视角
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();//更新ui->guiwidget上的显示
}

//主题切换类
class CommonHelper
{
public:
    static void setStyle(const QString &style) {
        QFile qss(style);
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
    }
};
//白蓝风格主题
void MainWindow::on_bb_theme_triggered()
{

    CommonHelper::setStyle(":/new/prefix1/resource/theme/white.qss");
}

//原风格主题
void MainWindow::on_action1_triggered()
{
    CommonHelper::setStyle(":/new/prefix1/resource/theme/no.qss");
}


//黑色风格主题
void MainWindow::on_action3_triggered()
{
    CommonHelper::setStyle(":/new/prefix1/resource/theme/QDarkStyleSheet.qss");
}

//白黑风格主题
void MainWindow::on_action4_triggered()
{
    CommonHelper::setStyle(":/new/prefix1/resource/theme/default.qss");
}

//点云可视化操作界面显示
void MainWindow::on_visualization_action_triggered()
{
    ui->dockWidget_2->show();

}

//点云处理操作界面显示
void MainWindow::on_processing_action_triggered()
{
    ui->dockWidget_3->show();
}

//清空树状图
void MainWindow::on_tree_clear_triggered()
{
    QStandardItemModel *model = qobject_cast<QStandardItemModel*>(ui->treeView->model());
    model->clear(); // 清除之前的内容
}



