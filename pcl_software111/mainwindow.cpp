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
#include <pcl/search/kdtree.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include<pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/concave_hull.h>
#include <QDesktopServices>

QQueue<LogElement> log_txt;
int count1=0;
int MeshId=0;

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
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    view.reset(new pcl::visualization::PCLVisualizer(renderer,renderWindow,"viewer",false));

    view->setupInteractor(ui->guiwidget->interactor(),ui->guiwidget->renderWindow());
    view->addCoordinateSystem(0.1);

    cloud_polygon.reset(new pcl::PointCloud<pcl::PointXYZ>);

    //设置回调函数

    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
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
    model = new QStandardItemModel(this);
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
    connect(ui->open_action,&QAction::triggered,this,[this]{
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
        fileItem = new QStandardItem();
        fileItem->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        fileItem->setCheckState(Qt::Checked); //设置复选框
        fileItem->setIcon(QIcon(":/new/prefix1/resource/cloud.png")); // 设置文件图标
        fileItem->setData(false, Qt::UserRole); // 存储不是目录的信息
        rootItem->appendRow(fileItem);
        ui->treeView->expandAll(); // 展开所有项
        ui->guiwidget->update();//确保更新ui->guiwidget上的显示


        int return_status=1;
        // 为每个文件创建新的点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr1(new pcl::PointCloud<pcl::PointXYZ>);
        if (filename.endsWith (".pcd", Qt::CaseInsensitive))
            {
            count1=count1+1;//为打开过的文件计数
            return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloudptr1);
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";

        }
        else if(filename.endsWith (".ply", Qt::CaseInsensitive)){
            count1=count1+1;//为打开过的文件计数
            return_status = pcl::io::loadPLYFile(filename.toStdString (), *cloudptr1);
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";
        }
        else if (filename.endsWith (".txt", Qt::CaseInsensitive))
        {
            cloudptr1->clear();
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
                cloudptr1->push_back(point);
            }
            return_status = 0;
            file.close();
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";
        }
        else
            {
            cloudptr1->clear();
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
                cloudptr1->push_back(point);
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
        int point_num = cloudptr1->size();
        QString point_num_str = QString::number(point_num);
        QString a = QString::number(count1);//转换类型，方便显示
        QStandardItem *pointcount = new QStandardItem("文件包含点的数量为："+point_num_str+"个");
        fileItem->appendRow(pointcount);

        // 将点云数据转换为VTK的点数据
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        for (size_t i = 0; i < cloudptr1->size(); ++i) {
            pcl::PointXYZ p = cloudptr1->at(i);
            points->InsertNextPoint(p.x, p.y, p.z);
        }
        cloud_id = std::to_string(count1);

        fileItem->setText(QString::fromStdString(cloud_id)+":"+QFileInfo(filePath).fileName());
        //计算质心
        // 创建一个Eigen::Vector4f对象来存储质心
        Eigen::Vector4f centroid1;
        bool centroid_computed = pcl::compute3DCentroid(*cloudptr1, centroid1);

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
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr1));
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(modifiedCloud);
        n.setInputCloud(modifiedCloud);
        n.setSearchMethod(tree);
        n.setKSearch(50);
        n.compute(*normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;
        gp3.setSearchRadius(10);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(150000);
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
    view->addPointCloud(cloudptr1,cloud_id);
    view->resetCamera();    //视角
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();//更新ui->guiwidget上的显示
    pointCloudMap.insert(QString::fromStdString(cloud_id), cloudptr1);
    pointCloudVisibilityMap.insert(QString::fromStdString(cloud_id), true);
    connect(model, &QStandardItemModel::itemChanged, this, &MainWindow::checkState);

    });

    //最近邻插值运算
    connect(ui->chazhi,&QPushButton::clicked,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行最近邻插值......";
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
            }
        }
        // 更新 pointCloudMap 中的指针
        pointCloudMap[cloudId] = cloudptr;
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"成功完成最近邻插值!";
    });

    //最近邻插值运算(图标)
    connect(ui->interpolation,&QAction::triggered,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行最近邻插值......";
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
            }
        }
        // 更新 pointCloudMap 中的指针
        pointCloudMap[cloudId] = cloudptr;
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"成功完成最近邻插值!";
    });


    //距离反比插值运算
    connect(ui->chazhi1,&QPushButton::clicked,this,[this]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行距离反比插值......";
        if (cloudptr) { // 检查 cloudptr2 是否为空
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
            // 创建 KdTree 对象进行最近邻搜索
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
            kdtree->setInputCloud(modifiedCloud);
            // 设置反比插值的幂次，这里使用 2 次幂
            const float power = 2.0f;

            // 对每个点进行搜索并插值
            for (size_t i = 0; i < modifiedCloud->points.size(); ++i) {
                std::vector<int> pointIdxNKNSearch;
                std::vector<float> pointNKNSquaredDistance;
                // 设置搜索的最近邻点数量，这里可能需要调整以获得更好的插值效果
                const int num_nn = 10; // 假设我们使用 10 个最近邻点进行插值
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
                }
            }

            // 更新 pointCloudMap 中的指针
            pointCloudMap[cloudId] = cloudptr;
            view->removeAllPointClouds();
            view->addPointCloud(cloudptr);
            ui->guiwidget->renderWindow()->Render();
            ui->guiwidget->update();
            qDebug() << "成功完成距离反比插值!";
        } else {
            qDebug() << "未找到当前显示的点云指针，无法进行距离反比插值操作。";
        }
    });

    //保存点云
    connect(ui->save_action,&QAction::triggered,this,[=]{
        if(MeshId==0)
        {
            // 获取当前显示的点云指针
            auto result = getCurrentlyDisplayedPointCloudFromView();
            QString cloudId = result.first;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
            qDebug()<<"正在将您的点云"<<cloudId<<"文件保存......";
            int return_status;
            QString filename = QFileDialog::getSaveFileName(this, tr("Save point cloud"), "", tr("Point cloud data (*.pcd *.ply *.stl)"));

            if (cloudptr->empty())
            {
                qDebug()<<"您未打开文件，无法进行保存操作！";
                return;
            }
            else
            {
                if (filename.isEmpty())
                {
                    qDebug()<<"保存失败，文件为空！";
                    return;
                }
                if (filename.endsWith(".pcd", Qt::CaseInsensitive))
                {
                    return_status = pcl::io::savePCDFileASCII(filename.toStdString(), *cloudptr);
                    qDebug()<<"已成功保存为PCD文件！";
                }
                else if (filename.endsWith(".ply", Qt::CaseInsensitive))
                {
                    return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
                    qDebug()<<"已成功保存为PLY文件！";
                }
                else if(filename.endsWith(".stl", Qt::CaseInsensitive))
                {
                    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
                    if (modifiedCloud->empty())
                    {
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
                }
                else
                {
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
                if (return_status != 0)
                {
                    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
                    qDebug()<<"保存失败，请重新操作！";
                    return;
                }
            }
        }
        else
        {
            pcl::PolygonMesh mesh = polygonMeshMap[QString::number(MeshId)];
            qDebug()<<"正在将您的点云"<<QString::number(MeshId)<<"文件保存......";
            int return_status;
            QString filename1 = QFileDialog::getSaveFileName(this, tr("Save point cloud"), "", tr("Point cloud data (*.pcd *.ply *.stl)"));
            if (filename1.isEmpty())
            {
                qDebug()<<"保存失败，文件为空！";
                return;
            }
            if(filename1.endsWith(".stl", Qt::CaseInsensitive))
            {
                pcl::io::savePolygonFileSTL(filename1.toStdString(), mesh);
                return_status = 0;
                qDebug()<<"已成功保存为stl文件！";
            }
            if (filename1.endsWith(".stl", Qt::CaseInsensitive)==false)
            {
                filename1.append(".stl");
            }
            else
            {
                filename1.append(".stl");
                pcl::io::savePolygonFileSTL(filename1.toStdString(), mesh);
                return_status = 0;
                qDebug()<<"已成功保存为stl文件！";
            }
            if (return_status != 0)
            {
                PCL_ERROR("Error writing point cloud %s\n", filename1.toStdString().c_str());
                qDebug()<<"保存失败，请重新操作！";
                return;
            }
        }
    });

    //转stl
    connect(ui->actionSTL, &QAction::triggered, this, [this]{
        if(MeshId==0)
        {
            // 获取当前显示的点云指针及其 cloudId
            auto result = getCurrentlyDisplayedPointCloudFromView();
            QString cloudId = result.first;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
            qDebug()<<"正在将文件"<<cloudId<<"另存为stl格式......";
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
        }
        else
        {
            pcl::PolygonMesh mesh = polygonMeshMap[QString::number(MeshId)];
            qDebug()<<"正在将文件"<<QString::number(MeshId)<<"另存为stl格式......";
            QString filename = QFileDialog::getSaveFileName(this, tr("Save STL File"), "", tr("STL Files (*.stl)"));
            if (filename.isEmpty())
            {
                qDebug()<<"转换失败，文件为空！";
                return;
            }
            if (filename.endsWith(".stl", Qt::CaseInsensitive)==false)
            {
                filename.append(".stl");
            }
            pcl::io::savePolygonFileSTL(filename.toStdString(), mesh);
            qDebug()<<"成功另存为stl格式！";
            return;
        }

    });

    //贪婪三角化并可视化
    connect(ui->TaLanSanJiao,&QPushButton::clicked,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        QElapsedTimer timer; //定义对象
        timer.start();  //开始计时
        qDebug()<<"正在对点云"<<cloudId<<"进行贪婪三角网格化......";
        // 创建点云副本，使用智能指针管理内存
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        if (modifiedCloud->empty()) {
            return;
        }

        //贪婪三角化变量获取
        float SearchRadius = ui->searchradius->value();
        float Fluent_Coefficient = ui->fluent_coefficient->value();
        int SetMaximumNearestNeighbors = ui->setMaximumNearestNeighbors->value();
        int index=ui->comboBox_3->currentIndex();

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
        gp3.setSearchRadius(SearchRadius);
        gp3.setMu(Fluent_Coefficient);
        gp3.setMaximumNearestNeighbors(SetMaximumNearestNeighbors);
        gp3.setMaximumSurfaceAngle(M_PI / 4);
        gp3.setMinimumAngle(M_PI / 36);
        gp3.setMaximumAngle(2 * M_PI / 3);
        if(index==0)
        {
            gp3.setNormalConsistency(false); // Reconstruct
        }
        else if(index==1)
        {
            gp3.setNormalConsistency(true);
        }
        else
        {
            qDebug()<<"软件出现致命错误！";
        }
        gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);
        polygonMeshMap[cloudId] = triangles;
        MeshId=cloudId.toInt();
        view->removeAllPointClouds();
        view->addPolygonMesh(triangles,cloudId.toStdString());
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"贪婪三角网格化处理完成！";
        qDebug() << "The operation took" << timer.elapsed() << "milliseconds"; //显示结果
    });

    //贪婪三角化并可视化（图标）
    connect(ui->Triangular_meshing,&QAction::triggered,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        QElapsedTimer timer; //定义对象
        timer.start();  //开始计时
        qDebug()<<"正在对点云"<<cloudId<<"进行贪婪三角网格化......";
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
        polygonMeshMap[cloudId] = triangles;
        MeshId=cloudId.toInt();
        view->removeAllPointClouds();
        view->addPolygonMesh(triangles,cloudId.toStdString());
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"贪婪三角网格化处理完成！";
        qDebug() << "The operation took" << timer.elapsed() << "milliseconds"; //显示结果
    });

    //泊松重建
    connect(ui->bosong,&QPushButton::clicked,this,[=]{
        QElapsedTimer timer; //定义对象
        timer.start();  //开始计时
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行泊松重建......";
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
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(8);  // 设置重建深度

        // 输入三角化后的点云数据（带法线）
        poisson.setInputCloud(cloud_with_normals);

        // 存储填补孔洞后的结果
        pcl::PolygonMesh recloud;
        poisson.reconstruct(recloud);
        polygonMeshMap[cloudId] = recloud;
        MeshId=cloudId.toInt();
        // 更新视图，显示填补后的模型
        view->removeAllPointClouds();
        view->addPolygonMesh(recloud, cloudId.toStdString());
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"泊松重建处理完成！";
        qDebug() << "The operation took" << timer.elapsed() << "milliseconds"; //显示结果
    });

    //拉普拉斯平滑处理
    connect(ui->pushButton_2,&QPushButton::clicked,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        int Diedai = ui->diedai->value();
        float Songchi = ui->songchi->value();
        qDebug()<<"正在对点云"<<cloudId<<"进行拉普拉斯平滑处理......";
        vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
        // 创建拉普拉斯平滑滤波器
        vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
            vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        smoothFilter->SetInputData(polydata);

        // 设置平滑迭代次数和松弛因子
        smoothFilter->SetNumberOfIterations(Diedai); // 迭代次数，根据需要进行调整
        smoothFilter->SetRelaxationFactor(Songchi); // 松弛因子，控制平滑的强度

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
        // 更新 pointCloudMap 中的指针
        pointCloudMap[cloudId] = cloudptr;
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"拉普拉斯平滑处理已完成！";

    });

    //高斯平滑处理
    connect(ui->Gauss_Button,&QPushButton::clicked,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行高斯平滑处理......";
        // 原始点云
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        float G_Searchradius=ui->G_searchradius->value();
        int G_Standard=ui->G_standard->value();
        // 卷积核
        pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;                            // 卷积核（用于高斯滤波）
        kernel.setSigma(G_Standard);                                                                           // 标准差
        kernel.setThresholdRelativeToSigma(4);                                                      // 关联作用域{可类比领域半径，通过公式计算}（用其自己的计算公式，公式为：||pi - q|| > sigma_coefficient^2 * sigma^2）
        kernel.setThreshold(10);                                                                    // 卷积半径
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);       // 进行领域搜索的kd树
        tree->setInputCloud(modifiedCloud);
        // 高斯滤波
        pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> filters;
        filters.setKernel(kernel);                                          // 设置卷积核
        filters.setInputCloud(modifiedCloud);                               // 输入点云
        filters.setNumberOfThreads(8);                                      // 同时计算的线程数
        filters.setSearchMethod(tree);                                      // 领域搜索的kd树
        filters.setRadiusSearch(G_Searchradius);                            // 搜索半径
        filters.convolve(*cloudptr);                                      // 计算

        // 更新 pointCloudMap 中的指针
        pointCloudMap[cloudId] = cloudptr;
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"高斯平滑处理已完成！";
    });

    //离群点去除
    connect(ui->delete_away,&QPushButton::clicked,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行离群点去除......";
        // 原始点云
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
        float L_Searchradius=ui->L_search->value();
        int L_Yuzhi=ui->L_yuzhi->value();
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(modifiedCloud);
        sor.setMeanK(L_Searchradius);
        sor.setStddevMulThresh(L_Yuzhi);
        sor.filter(*cloudptr);
        // 更新 pointCloudMap 中的指针
        pointCloudMap[cloudId] = cloudptr;
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"点云离群点去除已完成！";
    });

    //下采样
    connect(ui->pushButton,&QPushButton::clicked,this,[=]{
        // 获取当前显示的点云指针及其 cloudId
        auto result = getCurrentlyDisplayedPointCloudFromView();
        QString cloudId = result.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
        qDebug()<<"正在对点云"<<cloudId<<"进行下采样......";
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
        float leaf_size = min_distance * 0.8f;
        // 创建体素网格滤波器对象
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(modifiedCloud);
        // 设置体素网格的大小（以米为单位），这将决定下采样后的点云分辨率
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        // 应用滤波器并获取下采样后的点云
        sor.filter(*cloudptr);

        pointCloudMap[cloudId]=cloudptr;
        view->removeAllPointClouds();
        view->addPointCloud(cloudptr);
        ui->guiwidget->renderWindow()->Render();
        ui->guiwidget->update();
        qDebug()<<"下采样处理完成！";
    });

    //退出程序
    connect(ui->exit_action,&QAction::triggered,this,[=]{
        if (QMessageBox::question(this, tr("exit tip"), tr("你确定要退出吗 ?"), QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes)
        {
            qApp->quit();
        }
    });

    connect(ui->mergeButton, &QAction::triggered, this, &MainWindow::mergePointClouds);//拼接槽函数

}
MainWindow::~MainWindow()
{
    delete ui;
}


//查找当前界面正在显示的点云（得到其指针和Id）
std::pair<QString, pcl::PointCloud<pcl::PointXYZ>::Ptr> MainWindow::getCurrentlyDisplayedPointCloudFromView()
{
    vtkActorCollection* actorCollection = ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors();
    actorCollection->InitTraversal();
    vtkActor* actor = actorCollection->GetNextActor();
    while (actor) {
        vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
        if (polyData) {
            // 假设点云是 pcl::PointXYZ 类型
            pcl::PointCloud<pcl::PointXYZ>::Ptr searchcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::vtkPolyDataToPointCloud(polyData, *searchcloud);
            // 遍历 pointCloudMap 查找对应的 cloudId
            for (auto it = pointCloudMap.begin(); it!= pointCloudMap.end(); ++it) {
                if (arePointCloudsEqual(it.value(),searchcloud)) {
                    return std::make_pair(it.key(), searchcloud);
                }
            }
        }
        actor = actorCollection->GetNextActor();
    }
    return std::make_pair(QString(), nullptr);
}

//禁用功能实现函数
void MainWindow::setButtonsEnabled(bool enabled)
{

    // 更改ui为可用
    ui->actionMLS->setEnabled(enabled);
    ui->delete_away_2->setEnabled(enabled);
    ui->aobao->setEnabled(enabled);
    ui->action_B->setEnabled(enabled);
    ui->marching_cubes->setEnabled(enabled);
    ui->save_action->setEnabled(enabled);
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
    ui->chazhi->setEnabled(enabled);
    ui->chazhi1->setEnabled(enabled);
    ui->TaLanSanJiao->setEnabled(enabled);
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
    ui->Gauss_Button->setEnabled(enabled);
    ui->bosong->setEnabled(enabled);
    ui->delete_away->setEnabled(enabled);
    ui->Mouse_select->setEnabled(enabled);
    ui->divide->setEnabled(enabled);
    ui->tanlan->setEnabled(enabled);
    ui->bosong1->setEnabled(enabled);
    ui->actionStatisticalOutlierRemove->setEnabled(enabled);
    ui->menu_output->setEnabled(enabled);
    ui->VoxelGrid->setEnabled(enabled);
    ui->menu_3->setEnabled(enabled);
    ui->RadiusOutlinerRemoval->setEnabled(enabled);
    ui->StatisticalOutlierRemoval1->setEnabled(enabled);
    ui->ProjectInliers->setEnabled(enabled);
    ui->actionLaplas->setEnabled(enabled);
    ui->actionGauss->setEnabled(enabled);
    ui->actionSTL->setEnabled(enabled);
}

//控制点云是否显示
void MainWindow::checkState()
{
    // 递归遍历函数，用于遍历模型中的所有节点
    std::function<void(QModelIndex)> traverseItems = [this, &traverseItems](QModelIndex parentIndex) {
        int rowCount = model->rowCount(parentIndex);
        int columnCount = model->columnCount(parentIndex);

        for (int row = 0; row < rowCount; ++row) {
            for (int column = 0; column < columnCount; ++column) {
                QModelIndex currentIndex = model->index(row, column, parentIndex);
                QStandardItem* currentItem = model->itemFromIndex(currentIndex);

                if (currentItem && (currentItem->flags() & Qt::ItemIsUserCheckable)) {
                    QString text = currentItem->text();
                    QStringList parts = text.split(':');
                    QString beforeColon = parts.at(0);

                    if (currentItem->checkState() == Qt::Checked)
                    {
                        checkcount++;
                        if(row==0)
                        {
                        qDebug() << "正在显示点云: " << beforeColon;
                        }
                        //如果网格集合中包含数据，那么不显示点云集合中的数据，显示网格集合中的数据,如果两个集合中都没有数据，那就在点云集合中添加数据
                        if (pointCloudMap.contains(beforeColon) && !polygonMeshMap.contains(beforeColon))
                        {
                            auto cloudptr = pointCloudMap[beforeColon];
                            view->addPointCloud(pointCloudMap[beforeColon], beforeColon.toStdString());
                            pointCloudVisibilityMap[beforeColon] = true;
                        }
                        else if (polygonMeshMap.contains(beforeColon))
                        {
                            MeshId=beforeColon.toInt();
                            view->addPolygonMesh(polygonMeshMap[beforeColon], beforeColon.toStdString());
                            polygonMeshVisibilityMap[beforeColon] = true;
                        }
                        else //都不包含
                        {
                            qDebug() << "点云指针无效或无数据，无法添加点云";
                            if (allcloud && allcloud->size() > 0)
                            {
                                // 创建一个新的共享指针存储 cloudptr 的内容
                                auto newCloudPtr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*allcloud);
                                view->addPointCloud(newCloudPtr, beforeColon.toStdString());
                                pointCloudMap.insert(beforeColon, newCloudPtr);
                                pointCloudVisibilityMap.insert(beforeColon, true);
                            }
                            else
                            {
                                qDebug() << "点云指针无效或无数据，无法添加点云";
                            }
                        }
                    }
                    else//不勾选
                    {
                        if(row==0)
                        {
                            qDebug() << "正在隐藏点云: " << beforeColon;
                        }
                        if (polygonMeshMap.contains(beforeColon))
                        {
                            MeshId=0;
                            polygonMeshVisibilityMap[beforeColon] = false;
                            view->removePolygonMesh(beforeColon.toStdString());
                        }
                        else if (pointCloudMap.contains(beforeColon))
                        {
                            pointCloudVisibilityMap[beforeColon] = false;
                            view->removePointCloud(beforeColon.toStdString());
                        }
                    }
                    view->resetCamera();    //视角
                    ui->guiwidget->renderWindow()->Render();
                    ui->guiwidget->update();
                }

                // 递归调用，继续遍历当前节点的子节点（如果有的话）
                traverseItems(currentIndex);
            }
        }

    };

    checkcount = 0; //重置计数
    // 从不可见根节点开始遍历整个模型
    QModelIndex rootIndex = model->indexFromItem(model->invisibleRootItem());
    traverseItems(rootIndex);
    if(checkcount != 1) //如果复选框勾选数量大于1，则禁用其他大多数功能
    {
        ui->actionMLS->setEnabled(false);
        ui->delete_away_2->setEnabled(false);
        ui->aobao->setEnabled(false);
        ui->action_B->setEnabled(false);
        ui->marching_cubes->setEnabled(false);
        ui->Mouse_select->setEnabled(false);
        ui->save_action->setEnabled(false);
        ui->main_view->setEnabled(false);
        ui->back_view->setEnabled(false);
        ui->left_view->setEnabled(false);
        ui->right_view->setEnabled(false);
        ui->upward_view->setEnabled(false);
        ui->top_view->setEnabled(false);
        ui->render_begin->setEnabled(false);
        ui->comboBox->setEnabled(false);
        ui->comboBox_2->setEnabled(false);
        ui->main_view->setEnabled(false);
        ui->back_view->setEnabled(false);
        ui->left_view->setEnabled(false);
        ui->right_view->setEnabled(false);
        ui->upward_view->setEnabled(false);
        ui->top_view->setEnabled(false);
        ui->render_begin->setEnabled(false);
        ui->chazhi->setEnabled(false);
        ui->chazhi1->setEnabled(false);
        ui->TaLanSanJiao->setEnabled(false);
        ui->Triangular_meshing->setEnabled(false);
        ui->interpolation->setEnabled(false);
        ui->actionmain_view->setEnabled(false);
        ui->actionback_view->setEnabled(false);
        ui->actionleft_view->setEnabled(false);
        ui->actionright_view->setEnabled(false);
        ui->actionup_view->setEnabled(false);
        ui->actiontop_view->setEnabled(false);
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setEnabled(false);
        ui->Gauss_Button->setEnabled(false);
        ui->bosong->setEnabled(false);
        ui->delete_away->setEnabled(false);
        ui->Mouse_select->setEnabled(false);
        ui->divide->setEnabled(false);
        ui->tanlan->setEnabled(false);
        ui->bosong1->setEnabled(false);
        ui->actionStatisticalOutlierRemove->setEnabled(false);
        ui->menu_output->setEnabled(false);
        ui->VoxelGrid->setEnabled(false);
        ui->menu_3->setEnabled(false);
        ui->RadiusOutlinerRemoval->setEnabled(false);
        ui->StatisticalOutlierRemoval1->setEnabled(false);
        ui->ProjectInliers->setEnabled(false);
        ui->actionLaplas->setEnabled(false);
        ui->actionGauss->setEnabled(false);
        ui->actionSTL->setEnabled(false);
    }
    if(checkcount == 2)
    {
        ui->mergeButton->setEnabled(true);
    }
    else
    {
        ui->mergeButton->setEnabled(false);
    }
    setButtonsEnabled(checkcount == 1);//如果复选框勾选数量等于1，则启用其他大多数功能
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
    else if(index1==0)
    {
        // 从QVTKOpenGLWidget中获取vtkRenderWindow
        vtkRenderWindow* renderWindow = ui->guiwidget->renderWindow();
        // 获取renderer和actor
        vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
        vtkActor* actor = renderer->GetActors()->GetLastActor();
        // 创建VTK的高程渲染对象
        vtkSmartPointer<vtkElevationFilter> elevationFilter = vtkSmartPointer<vtkElevationFilter>::New();
        elevationFilter->SetInputConnection(actor->GetMapper()->GetInputConnection(0, 0));
        // 创建颜色映射器
        vtkSmartPointer<vtkColorTransferFunction> colorTransferFunction = vtkSmartPointer<vtkColorTransferFunction>::New();
        // 创建VTK的mapper和actor
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(elevationFilter->GetOutputPort());
        colorTransferFunction->AddRGBPoint(0.0, 1.0, 1.0, 1.0);
        colorTransferFunction->AddRGBPoint(1.0, 1.0, 1.0, 1.0);
        mapper->SetLookupTable(colorTransferFunction);
        actor->SetMapper(mapper);
        renderWindow->Render();
        qDebug()<<"已取消高程渲染！";
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

//关闭所有
void MainWindow::on_tree_clear_triggered()
{
    QStandardItemModel *model = qobject_cast<QStandardItemModel*>(ui->treeView->model());
    // 清除之前的内容
    model->clear();
    pointCloudMap.clear();
    pointCloudVisibilityMap.clear();
    polygonMeshMap.clear();
    polygonMeshVisibilityMap.clear();
    count1=0;
    view->removeAllPointClouds();
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"已关闭所有文件！";
}

//坐标转换函数（屏幕坐标转世界坐标）
void MainWindow::getScreentPos(double* displayPos, double* world, void* viewer_void)
{
    qDebug()<<"执行屏幕坐标转换至世界坐标函数";
    pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    vtkRenderer* renderer{viewer->getRendererCollection()->GetFirstRenderer()};
    double fp[4], tmp1[4], eventFPpos[4];
    renderer->GetActiveCamera()->GetFocalPoint(fp);
    fp[3] = 0.0;
    renderer->SetWorldPoint(fp);
    renderer->WorldToDisplay();
    renderer->GetDisplayPoint(tmp1);

    tmp1[0] = displayPos[0];
    tmp1[1] = displayPos[1];

    renderer->SetDisplayPoint(tmp1);
    renderer->DisplayToWorld();
    renderer->GetWorldPoint(eventFPpos);

    for (int i = 0; i < 3; i++)
    {
        world[i] = eventFPpos[i];
    }
}

//点云分割判断函数
int MainWindow::inOrNot1(int poly_sides, double *poly_X, double *poly_Y, double x, double y)
{
    int i, j;
    j = poly_sides - 1;
    int res = 0;

    for (i = 0; i < poly_sides; i++)
    {
        if (((poly_Y[i] < y && poly_Y[j] >= y) || (poly_Y[j] < y && poly_Y[i] >= y)) && (poly_X[i] <= x || poly_X[j] <= x))
        {
            res ^= ((poly_X[i] + (y - poly_Y[i]) / (poly_Y[j] - poly_Y[i])  *(poly_X[j] - poly_X[i])) < x);
        }
        j = i;
    }
    return res;
}

//点云分割函数
void MainWindow::projectInliers(void* viewer_void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped2(new pcl::PointCloud<pcl::PointXYZ>);
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    int id = cloudId.toInt();
    qDebug()<<cloudId;
    pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    // 存储焦点和相机位置的数组
    double focal[3] = {0};
    double pos[3] = {0};
    vtkRenderer* renderer{viewer->getRendererCollection()->GetFirstRenderer()};// 获取渲染器
    renderer->GetActiveCamera()->GetFocalPoint(focal);//获取相机的焦点
    renderer->GetActiveCamera()->GetPosition(pos);//获取相机的位置
    //输出
    std::cout << "focal: " << focal[0] << ',' << focal[1] << ',' << focal[2] << endl;
    std::cout << "pos: " << pos[0] << ',' << pos[1] << ',' << pos[2] << endl;
    pcl::PointXYZ eyeLine1 = pcl::PointXYZ(focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]);// 计算从相机位置到焦点的向量
    float mochang = sqrt(pow(eyeLine1.x, 2) + pow(eyeLine1.y, 2) + pow(eyeLine1.z, 2));//计算向量的模长
    pcl::PointXYZ eyeLine = pcl::PointXYZ(eyeLine1.x / mochang, eyeLine1.y / mochang, eyeLine1.z / mochang);// 归一化该向量，得到单位向量
    // 平面方程 ax + by + cz + d = 0，这里设置平面的法向量和 d = 0
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = eyeLine.x;
    coefficients->values[1] = eyeLine.y;
    coefficients->values[2] = eyeLine.z;
    coefficients->values[3] = 0;
    // 创建存储投影结果的点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_Prj(new pcl::PointCloud<pcl::PointXYZ>);//存储点云的投影结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCiecle_result(new pcl::PointCloud<pcl::PointXYZ>);//存储多边形的投影结果
    pcl::ProjectInliers<pcl::PointXYZ> proj;// 创建投影滤波器对象
    proj.setModelType(pcl::SACMODEL_PLANE);//设置投影模型为平面
    proj.setInputCloud(cloud_polygon);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloudCiecle_result);

    pcl::ProjectInliers<pcl::PointXYZ> projCloudIn;// 创建投影滤波器对象
    projCloudIn.setModelType(pcl::SACMODEL_PLANE);//设置投影模型为平面
    projCloudIn.setInputCloud(cloudptr);
    projCloudIn.setModelCoefficients(coefficients);
    projCloudIn.filter(*cloudIn_Prj);

    int ret = -1;// 存储判断结果的变量
    double *PloyXarr = new double[cloudCiecle_result->points.size()];// 存储投影结果的多边形顶点的 x、y坐标
    double *PloyYarr = new double[cloudCiecle_result->points.size()];
    for (int i = 0; i < cloudCiecle_result->points.size(); i++)
    {
        PloyXarr[i] = cloudCiecle_result->points[i].x;
        PloyYarr[i] = cloudCiecle_result->points[i].y;
    }

    for (int i = 0; i < cloudIn_Prj->points.size(); i++)
    {
        //判断点云是否在多边形内
        ret = inOrNot1(cloud_polygon->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
        if (1 == ret)
        {
            // 如果点在多边形内，将该点添加到 cloud_cliped1 中
            cloud_cliped1->points.push_back(cloudptr->points[i]);
        }
        else
        {
            // 如果点不在多边形内，将该点添加到 cloud_cliped2 中
            cloud_cliped2->points.push_back(cloudptr->points[i]);
        }
    }

    id=id+1;
    count1=count1+1;
    viewer->addPointCloud(cloud_cliped1,std::to_string(count1));
    pointCloudMap.insert(QString::number(count1), cloud_cliped1);
    pointCloudVisibilityMap.insert(QString::number(count1), true);

    //保留的点云项
    QStandardItemModel *model = qobject_cast<QStandardItemModel*>(ui->treeView->model());

    if (!model) {
        model = new QStandardItemModel(this);
        ui->treeView->setModel(model);
    }
    QStandardItem* fileItem1 = new QStandardItem();
    fileItem1->setIcon(QIcon(":/new/prefix1/resource/cloud.png"));
    fileItem1->setData(false, Qt::UserRole);
    fileItem1->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    fileItem1->setCheckState(Qt::Checked);
    fileItem1->setText(QString::number(count1)+":"+"点云"+cloudId+"的分割部分1");
    //创建保留点云项的各节点并添加
    QStandardItem* rootItem1 = model->invisibleRootItem();
    QStandardItem* parentItem1 = new QStandardItem();
    parentItem1->setIcon(QIcon( ":/new/prefix1/resource/file.png")); // 设置文件图标
    parentItem1->setData(true, Qt::UserRole); // 存储是目录的信息
    parentItem1->setText("点云"+cloudId+"分割结果");
    rootItem1->appendRow(parentItem1);
    rootItem1=parentItem1;
    rootItem1->appendRow(fileItem1);
    //去除的点云项
    id=id+1;
    count1=count1+1;
    viewer->addPointCloud(cloud_cliped2,std::to_string(count1));
    pointCloudMap.insert(QString::number(count1), cloud_cliped2);
    pointCloudVisibilityMap.insert(QString::number(count1), true);
    QStandardItem* fileItem2 = new QStandardItem();
    fileItem2->setIcon(QIcon(":/new/prefix1/resource/cloud.png"));
    fileItem2->setData(false, Qt::UserRole);
    fileItem2->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    fileItem2->setCheckState(Qt::Checked);
    fileItem2->setText(QString::number(count1)+":"+"点云"+cloudId+"的分割部分2");
    //创建去除点云项的各节点并添加
    QStandardItem* rootItem2 = model->invisibleRootItem();
    QStandardItem* parentItem2 = new QStandardItem();
    parentItem2->setIcon(QIcon(":/new/prefix1/resource/file.png")); // 设置文件图标
    parentItem2->setData(true, Qt::UserRole); // 存储是目录的信息
    parentItem2->setText("点云"+cloudId+"分割结果");
    rootItem2->appendRow(parentItem2);
    rootItem2=parentItem2;
    rootItem2->appendRow(fileItem2);
    ui->treeView->expandAll(); // 展开所有项

    //更新显示界面并输出Debug
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"点云分割已完成！";
    connect(model, &QStandardItemModel::itemChanged, this, &MainWindow::checkState);
}

//鼠标框选启动函数
void MainWindow::on_Mouse_select_triggered()
{
    qDebug()<<"正在对要分割的点云进行鼠标框选......";
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    ui->guiwidget->interactor()->SetInteractorStyle(style);
    ui->guiwidget->setFocus();
    isPickingMode =!isPickingMode;
    if (isPickingMode)
    {
        qDebug()<< "start draw";
        line_id = 0;
        cloud_polygon->clear();
        flag = false;
        ui->guiwidget->interactor()->AddObserver(vtkCommand::LeftButtonReleaseEvent, this, &MainWindow::mouseEventOccurred);
    }
    else
    {
        qDebug()<<"当前未进入鼠标框选模式！";
    }
}

//鼠标事件函数
void MainWindow::mouseEventOccurred(vtkObject* caller, unsigned long eventId, void* callData)
{
    (void)callData; // 显式地表示不使用该参数
    if (eventId == vtkCommand::LeftButtonReleaseEvent)
    {
        vtkRenderWindowInteractor* interactor = vtkRenderWindowInteractor::SafeDownCast(caller);
        if (interactor)
        {
            pcl::visualization::PCLVisualizer* viewer = view.get();

            viewer->addPointCloud(cloud_polygon, "polyline");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0, 0, 0, "polyline");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "polyline");


            int* clickPos = interactor->GetEventPosition();
            std::cout << "Left mouse button released at position (" << clickPos[0] << ", " << clickPos[1] << ")" << std::endl;


            // 检查是否处于选择模式（isPickingMode 为 true）
            if (isPickingMode)
            {
                double world_point[3];  // 世界坐标的数组
                double displayPos[2];  // 屏幕坐标的数组
                displayPos[0] = static_cast<double>(clickPos[0]);  // 将鼠标事件的屏幕坐标转换为 double 类型并存入 displayPos 数组
                displayPos[1] = static_cast<double>(clickPos[1]);


                // 假设你有一个将屏幕坐标转换为世界坐标的函数，例如 getScreentPos
                getScreentPos(displayPos, world_point, viewer);


                // 输出世界坐标
                std::cout << world_point[0] << ',' << world_point[1] << ',' << world_point[2] << std::endl;


                // 将世界坐标转换为 pcl::PointXYZ 类型的点
                curP = pcl::PointXYZ(world_point[0], world_point[1], world_point[2]);


                // 如果 flag 为 false，说明是第一次点击，将 flag 置为 true
                if (!flag)
                    flag = true;
                else
                {
                    char str1[512];
                    sprintf(str1, "line#%03d", line_id++);
                    viewer->addLine(lastP,curP, str1);
                }
                lastP = curP;
                cloud_polygon->push_back(curP);
                qDebug() << "成功设定框选边！";
                // 更新渲染窗口
                ui->guiwidget->renderWindow()->Render();
                ui->guiwidget->update();
            }
        }
    }

}

//点云分割启动函数
void MainWindow::on_divide_triggered()
{
    qDebug()<<"开始进行点云分割......";
    isPickingMode =!isPickingMode;
    qDebug()<< "stop draw";
    projectInliers(view.get());
    view->removeAllShapes();
}

//贪婪三角化（菜单栏）
void MainWindow::on_tanlan_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行贪婪三角网格化......";
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
    polygonMeshMap[cloudId] = triangles;
    MeshId=cloudId.toInt();
    view->removeAllPointClouds();
    view->addPolygonMesh(triangles,cloudId.toStdString());
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"贪婪三角网格化处理完成！";
}

//泊松重建（菜单栏）
void MainWindow::on_bosong1_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行泊松重建......";
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
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(8);  // 设置重建深度

    // 输入三角化后的点云数据（带法线）
    poisson.setInputCloud(cloud_with_normals);

    // 存储填补孔洞后的结果
    pcl::PolygonMesh recloud;
    poisson.reconstruct(recloud);
    polygonMeshMap[cloudId] = recloud;
    MeshId=cloudId.toInt();
    // 更新视图，显示填补后的模型
    view->removeAllPointClouds();
    view->addPolygonMesh(recloud, cloudId.toStdString());
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"泊松重建处理完成！";
}

//离群点去除（菜单栏）
void MainWindow::on_actionStatisticalOutlierRemove_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行离群点去除......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    float L_Searchradius=ui->L_search->value();
    int L_Yuzhi=ui->L_yuzhi->value();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(modifiedCloud);
    sor.setMeanK(L_Searchradius);
    sor.setStddevMulThresh(L_Yuzhi);
    sor.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"点云离群点去除已完成！";
}

//另存为PLY格式
void MainWindow::on_actionPLY_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在将文件"<<cloudId<<"另存为PLY格式......";
    QString filename = QFileDialog::getSaveFileName(this, tr("Save PLY File"), "", tr("PLY Files (*.ply)"));
    if (filename.isEmpty()) {
        qDebug()<<"另存PLY失败，文件为空！";
        return;
    }
    if (cloudptr->empty()) {
        qDebug()<<"另存失败，未找到用于转换的点云！";
        return;
    }
    if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
        pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
        qDebug()<<"成功另存为PLY格式！";
    }
    else if(filename.endsWith(".ply", Qt::CaseInsensitive)==false) {
        filename.append(".ply");
        pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
        qDebug()<<"成功另存为PLY格式！";
    }
    return;
}

//另存为PCD格式
void MainWindow::on_actionPCD_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在将文件"<<cloudId<<"另存为PCD格式......";
    QString filename = QFileDialog::getSaveFileName(this, tr("Save PCD File"), "", tr("PCD Files (*.pcd)"));
    if (filename.isEmpty()) {
        qDebug()<<"另存PCD失败，文件为空！";
        return;
    }
    if (cloudptr->empty()) {
        qDebug()<<"另存失败，未找到用于转换的点云！";
        return;
    }
    if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
        pcl::io::savePCDFileASCII(filename.toStdString(), *cloudptr);
        qDebug()<<"成功另存为PCD格式！";
    }
    else if(filename.endsWith(".pcd", Qt::CaseInsensitive)==false) {
        filename.append(".pcd");
        pcl::io::savePCDFileASCII(filename.toStdString(), *cloudptr);
        qDebug()<<"成功另存为PCD格式！";
    }
    return;
}

//另存为TXT格式
void MainWindow::on_actionTXT_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在将文件"<<cloudId<<"另存为TXT格式......";
    QString filename = QFileDialog::getSaveFileName(this, tr("Save TXT File"), "", tr("TXT Files (*.txt)"));
    if (filename.isEmpty()) {
        qDebug()<<"另存TXT失败，文件为空！";
        return;
    }
    if (cloudptr->empty()) {
        qDebug()<<"另存失败，未找到用于转换的点云！";
        return;
    }
    if (filename.endsWith(".txt", Qt::CaseInsensitive)) {
        // 创建文件输出流
        std::ofstream outfile(filename.toStdString());
        if (!outfile.is_open())
        {
            qDebug()<<"无法打开文件！";
            return;
        }
        // 遍历点云数据并写入文件
        for (const auto& point : cloudptr->points)
        {
            outfile << point.x << " " << point.y << " " << point.z << std::endl;
        }
        // 关闭文件输出流
        outfile.close();
        qDebug()<<"成功另存为TXT格式！";
    }
    else if(filename.endsWith(".txt", Qt::CaseInsensitive)==false) {
        filename.append(".txt");
        // 创建文件输出流
        std::ofstream outfile(filename.toStdString());
        if (!outfile.is_open())
        {
            qDebug()<<"无法打开文件！";
            return;
        }
        // 遍历点云数据并写入文件
        for (const auto& point : cloudptr->points)
        {
            outfile << point.x << " " << point.y << " " << point.z << std::endl;
        }
        // 关闭文件输出流
        outfile.close();
        qDebug()<<"成功另存为TXT格式！";
    }
    return;
}

//另存为OBJ格式
void MainWindow::on_actionOBJ_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在将文件"<<cloudId<<"另存为OBJ格式......";
    QString filename = QFileDialog::getSaveFileName(this, tr("Save OBJ File"), "", tr("OBJ Files (*.obj)"));
    if (filename.isEmpty()) {
        qDebug()<<"另存OBJ失败，文件为空！";
        return;
    }
    if (cloudptr->empty()) {
        qDebug()<<"另存失败，未找到用于转换的点云！";
        return;
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
    gp3.setSearchRadius(10);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(1500);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 36);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals); gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    if (filename.endsWith(".obj", Qt::CaseInsensitive)) {
        // 创建文件输出流
        std::ofstream outfile(filename.toStdString());
        if (!outfile.is_open())
        {
            qDebug()<<"无法打开文件！";
            return;
        }
        // 存储顶点信息
        for (const auto& point : cloud_with_normals->points)
        {
            outfile << "v " << point.x << " " << point.y << " " << point.z << std::endl;
        }
        // 存储法线信息
        for (const auto& point : cloud_with_normals->points)
        {
            outfile << "vn " << point.normal_x << " " << point.normal_y << " " << point.normal_z << std::endl;
        }
        // 存储面信息
        for (const auto& polygon : triangles.polygons)
        {
            outfile << "f ";
            for (const auto index : polygon.vertices)
            {
                outfile << index + 1 << " ";
            }
            outfile << std::endl;
        }
        // 关闭文件输出流
        outfile.close();
        qDebug()<<"成功另存为obj格式！";
    }
    else if(filename.endsWith(".txt", Qt::CaseInsensitive)==false) {
        filename.append(".obj");
        // 创建文件输出流
        std::ofstream outfile(filename.toStdString());
        if (!outfile.is_open())
        {
            qDebug()<<"无法打开文件！";
            return;
        }
        // 存储顶点信息
        for (const auto& point : cloud_with_normals->points)
        {
            outfile << "v " << point.x << " " << point.y << " " << point.z << std::endl;
        }
        // 存储法线信息
        for (const auto& point : cloud_with_normals->points)
        {
            outfile << "vn " << point.normal_x << " " << point.normal_y << " " << point.normal_z << std::endl;
        }
        // 存储面信息
        for (const auto& polygon : triangles.polygons)
        {
            outfile << "f ";
            for (const auto index : polygon.vertices)
            {
                outfile << index + 1 << " ";
            }
            outfile << std::endl;
        }
        outfile.close();
        qDebug()<<"成功另存为obj格式！";
    }
    return;
}

//关闭所有
void MainWindow::on_close_action_triggered()
{
    QStandardItemModel *model = qobject_cast<QStandardItemModel*>(ui->treeView->model());
    // 清除之前的内容
    model->clear();
    pointCloudMap.clear();
    pointCloudVisibilityMap.clear();
    polygonMeshMap.clear();
    polygonMeshVisibilityMap.clear();
    count1=0;
    view->removeAllPointClouds();
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"已关闭所有文件！";
}

//统计离群点去除滤波器
void MainWindow::on_StatisticalOutlierRemoval1_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行统计离群点去除......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(modifiedCloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.00);
    sor.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"点云统计离群点去除已完成！";
}

//体素滤波器
void MainWindow::on_VoxelGrid_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行体素滤波......";
    // 创建点云副本，使用智能指针管理内存
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    if (modifiedCloud->empty()) {
        qDebug()<<"体素滤波失败！";
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
    float leaf_size = min_distance * 0.8f;
    // 创建体素网格滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(modifiedCloud);
    // 设置体素网格的大小（以米为单位），这将决定体素滤波后的点云分辨率
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    // 应用滤波器并获取体素滤波后的点云
    sor.filter(*cloudptr);

    pointCloudMap[cloudId]=cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"体素滤波处理完成！";
}

//半径离群点去除滤波器
void MainWindow::on_RadiusOutlinerRemoval_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行半径离群点去除......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
    pcFilter.setInputCloud(modifiedCloud);              //设置待滤波的点云
    pcFilter.setRadiusSearch(0.8);                      // 设置搜索半径
    pcFilter.setMinNeighborsInRadius(2);                // 设置一个内点最少的邻居数目
    pcFilter.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"点云半径离群点去除已完成！";
}

//直通滤波器X轴
void MainWindow::on_actionX_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行X轴直通滤波......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(modifiedCloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 1.0); //闭区间[0.0, 1.0]，保留该闭区间内的值
    pass.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"X轴点云直通滤波已完成！";
}

//直通滤波器Y轴
void MainWindow::on_actiony_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行Y轴直通滤波......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(modifiedCloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 1.0); //闭区间[0.0, 1.0]，保留该闭区间内的值
    pass.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"Y轴点云直通滤波已完成！";
}

//直通滤波器Z轴
void MainWindow::on_actionZ_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行Z轴直通滤波......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(modifiedCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0); //闭区间[0.0, 1.0]，保留该闭区间内的值
    pass.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"Z轴点云直通滤波已完成！";
}

//投影参数化模型滤波器
void MainWindow::on_ProjectInliers_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行投影参数化模型滤波......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    //平面 ax+by+cz+d=0 的系数a b c d
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients());
    coef->values.resize(4);
    coef->values[0] = 1;
    coef->values[1] = 1;
    coef->values[2] = 1;
    coef->values[3] = -1;
    //ProjectInliers滤波器
    pcl::ProjectInliers<pcl::PointXYZ> prot;
    prot.setModelType(pcl::SACMODEL_PLANE); //设置投影模型为平面模型
    prot.setInputCloud(modifiedCloud);
    prot.setModelCoefficients(coef);
    prot.filter(*cloudptr);
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"投影参数化模型滤波已完成！";
}

//拉普拉斯平滑（图标）
void MainWindow::on_actionLaplas_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行拉普拉斯平滑处理......";
    vtkPolyData* polydata = vtkPolyData::SafeDownCast(ui->guiwidget->renderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetMapper()->GetInput());
    // 创建拉普拉斯平滑滤波器
    vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
        vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
    smoothFilter->SetInputData(polydata);

    // 设置平滑迭代次数和松弛因子
    smoothFilter->SetNumberOfIterations(10); // 迭代次数，根据需要进行调整
    smoothFilter->SetRelaxationFactor(0.50); // 松弛因子，控制平滑的强度

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
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"拉普拉斯平滑处理已完成！";

}

//高斯平滑（图标）
void MainWindow::on_actionGauss_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行高斯平滑处理......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    // 卷积核
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;                            // 卷积核（用于高斯滤波）
    kernel.setSigma(6);                                                                           // 标准差
    kernel.setThresholdRelativeToSigma(4);                                                      // 关联作用域{可类比领域半径，通过公式计算}（用其自己的计算公式，公式为：||pi - q|| > sigma_coefficient^2 * sigma^2）
    kernel.setThreshold(10);                                                                    // 卷积半径
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);       // 进行领域搜索的kd树
    tree->setInputCloud(modifiedCloud);
    // 高斯滤波
    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> filters;
    filters.setKernel(kernel);                                          // 设置卷积核
    filters.setInputCloud(modifiedCloud);                               // 输入点云
    filters.setNumberOfThreads(8);                                      // 同时计算的线程数
    filters.setSearchMethod(tree);                                      // 领域搜索的kd树
    filters.setRadiusSearch(10.00);                                     // 搜索半径
    filters.convolve(*cloudptr);                                        // 计算

    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"高斯平滑处理已完成！";
}

//移动立方体重建
void MainWindow::on_marching_cubes_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行移动立方体重建......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));

    //1  法线估计
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(modifiedCloud);
    n.setInputCloud(modifiedCloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //2  法线点云
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*modifiedCloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    //3  KD索引
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    //4 移动立方体
    pcl::MarchingCubes<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
    mc->setIsoLevel(0.5f);
    mc->setGridResolution(30, 30, 30);
    mc->setPercentageExtendGrid(0.1f);
    mc->setSearchMethod(tree2);
    mc->setInputCloud(cloud_with_normals);
    pcl::PolygonMesh mesh;//结果
    mc->reconstruct(mesh);
    polygonMeshMap[cloudId] = mesh;
    MeshId=cloudId.toInt();
    view->removeAllPointClouds();
    view->addPolygonMesh(mesh,cloudId.toStdString());
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"移动立方体重建完成！";

}

//a-shape凹包重建（菜单栏、图标）
void MainWindow::on_action_B_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行a-shape凹包重建......";
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(modifiedCloud);
    //设置 alpha 值
    float alpha = 0.5; // 可以根据需要调整 alpha 值
    concave_hull.setAlpha(alpha);
    //执行凹包计算
    pcl::PolygonMesh mesh;
    concave_hull.reconstruct(mesh);
    polygonMeshMap[cloudId] = mesh;
    MeshId=cloudId.toInt();
    view->removeAllPointClouds();
    view->addPolygonMesh(mesh,cloudId.toStdString());
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"a-shape凹包重建完成！";
}

//凹包算法（界面）
void MainWindow::on_aobao_clicked()
{
    float t = ui->searchradius_2->value();
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行a-shape凹包重建......";
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(modifiedCloud);
    concave_hull.setAlpha(t);
    //执行凹包计算
    pcl::PolygonMesh mesh;
    concave_hull.reconstruct(mesh);
    polygonMeshMap[cloudId] = mesh;
    MeshId=cloudId.toInt();
    view->removeAllPointClouds();
    view->addPolygonMesh(mesh,cloudId.toStdString());
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"a-shape凹包重建完成！";
}

//使用指南打开
void MainWindow::on_action_help_triggered()
{
    qDebug()<<"正在打开使用指南！";
    QString filePath = "E:/pcl_software111/using_help_file.pdf";
    QUrl url(QUrl::fromLocalFile(filePath));
    QDesktopServices::openUrl(url);
    qDebug()<<"使用指南已为您打开！";
}

//快捷键表打开
void MainWindow::on_action_quick_triggered()
{
    qDebug()<<"正在打开快捷键表！";
    QString filePath = "E:/pcl_software111/Shortcut_list.pdf";
    QUrl url(QUrl::fromLocalFile(filePath));
    QDesktopServices::openUrl(url);
    qDebug()<<"快捷键表已为您打开！";
}

//点云拼接
void MainWindow::mergePointClouds()
{
    // 获取分割后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped1 = pointCloudMap.value(QString::number(count1 - 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped2 = pointCloudMap.value(QString::number(count1));

    if (!cloud_cliped1 || !cloud_cliped2)
    {
        qDebug() << "找不到分割后的点云，无法拼接！";
        return;
    }

    // 合并点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    *mergedCloud = *cloud_cliped1 + *cloud_cliped2;

    // 添加合并后的点云到 PCLVisualizer
    count1++;
    pcl::visualization::PCLVisualizer* viewer = view.get();
    viewer->addPointCloud(mergedCloud, std::to_string(count1));
    pointCloudMap.insert(QString::number(count1), mergedCloud);
    pointCloudVisibilityMap.insert(QString::number(count1), true);

    // 在树状控件中添加条目
    QStandardItemModel* model = qobject_cast<QStandardItemModel*>(ui->treeView->model());
    if (!model)
    {
        model = new QStandardItemModel(this);
        ui->treeView->setModel(model);
    }

    QStandardItem* fileItem = new QStandardItem();
    fileItem->setIcon(QIcon(":/new/prefix1/resource/cloud.png"));
    fileItem->setData(false, Qt::UserRole);
    fileItem->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    fileItem->setCheckState(Qt::Checked);
    fileItem->setText(QString::number(count1) + ":拼接后的点云");

    QStandardItem* rootItem = model->invisibleRootItem();
    QStandardItem* parentItem = new QStandardItem();
    parentItem->setIcon(QIcon(":/new/prefix1/resource/file.png"));
    parentItem->setData(true, Qt::UserRole);
    parentItem->setText("拼接结果");
    rootItem->appendRow(parentItem);
    rootItem = parentItem;
    rootItem->appendRow(fileItem);

    // 更新显示
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    ui->treeView->expandAll();

    qDebug() << "点云拼接已完成！";
}

//MLS平滑（图标）
void MainWindow::on_actionMLS_triggered()
{
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行MLS平滑处理......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(modifiedCloud);
    mls.setComputeNormals(false);					// 是否计算法线，设置为ture则计算法线
    mls.setPolynomialOrder(2);						// 设置MLS拟合的阶数，默认是2
    mls.setSearchMethod(tree);						// 邻域点搜索的方式
    mls.setSearchRadius(10);				    // 邻域搜索半径
    mls.setNumberOfThreads(10);						// 设置多线程加速的线程数
    mls.process(*cloudptr);				            // 曲面重建
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"MLS平滑处理已完成！";
}

//MLS平滑
void MainWindow::on_delete_away_2_clicked()
{
    int PolynomialOrder = ui->L_search_2->value();
    int SearchRadius = ui->L_search_3->value();
    // 获取当前显示的点云指针及其 cloudId
    auto result = getCurrentlyDisplayedPointCloudFromView();
    QString cloudId = result.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr = result.second;
    qDebug()<<"正在对点云"<<cloudId<<"进行MLS平滑处理......";
    // 原始点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modifiedCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudptr));
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(modifiedCloud);
    mls.setComputeNormals(false);					// 是否计算法线，设置为ture则计算法线
    mls.setPolynomialOrder(PolynomialOrder);						// 设置MLS拟合的阶数，默认是2
    mls.setSearchMethod(tree);						// 邻域点搜索的方式
    mls.setSearchRadius(SearchRadius);				    // 邻域搜索半径
    mls.setNumberOfThreads(10);						// 设置多线程加速的线程数
    mls.process(*cloudptr);				            // 曲面重建
    // 更新 pointCloudMap 中的指针
    pointCloudMap[cloudId] = cloudptr;
    view->removeAllPointClouds();
    view->addPointCloud(cloudptr);
    ui->guiwidget->renderWindow()->Render();
    ui->guiwidget->update();
    qDebug()<<"MLS平滑处理已完成！";
}

