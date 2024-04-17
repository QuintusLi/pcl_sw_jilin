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

QQueue<LogElement> log_txt;
int count1=0;
MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent), ui(new Ui::MainWindow), buttonsEnabled(false)
{
    ui->setupUi(this);
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
    ui->guiwidget->setRenderWindow(view->getRenderWindow());
    qDebug()<<"可视化窗口初始化成功！可以进行操作！";




    //打开点云文件的匿名信号槽
    connect(ui->open_action,&QAction::triggered,this,[cloudptr,this]{
        // 首先尝试从视图中移除之前的点云
        view->removeAllPointClouds();
        //读取pcd文件并显示
        QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "", tr ("Point cloud data (*.txt *.pcd *.ply)"));
        PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

        if (filename.isEmpty ()){
            qDebug()<<"打开失败，文件为空！";
            return;}


        ui->guiwidget->update();//确保更新ui->guiwidget上的显示

        int return_status=1;
        if (filename.endsWith (".pcd", Qt::CaseInsensitive))
            {
            qDebug()<<"正在打开文件:"<<filename.toStdString ().c_str ()<<"......";
            count1=count1+1;//为打开过的文件计数
            return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloudptr);
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";

        }
        else if (filename.endsWith (".txt", Qt::CaseInsensitive))
        {
            qDebug()<<"正在打开文件:"<<filename.toStdString ().c_str ()<<"......";
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
                return_status = 0;
            }
            file.close();
            qDebug()<<"成功打开文件:"<<filename.toStdString ().c_str ()<<"!";
        }
        else
            {
            qDebug()<<"正在打开文件:"<<filename.toStdString ().c_str ()<<"......";
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
                return_status = 0;
            }
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
        // 获取点云中点的数量并将其转换为字符串
        int point_num = cloudptr->size();
        QString point_num_str = QString::number(point_num);
        QString a = QString::number(count1);//转换类型，方便显示
        ui->textBrowser->append(a+". "+point_num_str+"个");// 在textbrowser中显示点云中点的数量
        ui->textEdit->append(a+". "+filename);//在textedit中显示文件名称
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
                qDebug()<<"正在将您的点云文件保存为pcd格式......";
                return_status = pcl::io::savePCDFileASCII(filename.toStdString(), *cloudptr);
                qDebug()<<"保存成功！";
            } else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
                qDebug()<<"正在将您的点云文件保存为ply格式......";
                return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
                qDebug()<<"保存成功！";
            } else if(filename.endsWith(".stl", Qt::CaseInsensitive)) {
                qDebug()<<"正在将您的点云文件保存为stl格式......";
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
                gp3.reconstruct(triangles); // Save STL file pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
                pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
                return_status = 0;
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
                qDebug()<<"保存成功！";
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
        QString filename = QFileDialog::getSaveFileName(this, tr("Save STL File"), "", tr("STL Files (*.stl)"));
        if (filename.isEmpty()) {
            return;
        }
        if (filename.endsWith(".stl", Qt::CaseInsensitive)) {
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
            gp3.reconstruct(triangles); // Save STL file pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
            pcl::io::savePolygonFileSTL(filename.toStdString(), triangles);
            return;
        }


    });

    //贪婪三角化并可视化
    connect(ui->TaLanSanJiao,&QPushButton::clicked,this,[=]{
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
    });




    //退出程序
    connect(ui->exit_action,&QAction::triggered,this,[=]{
        if (!(QMessageBox::information(this,tr("exit tip"),tr("你确定要退出吗 ?"),tr("Yes"),tr("No"))))
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
    ui->textEdit->setEnabled(enabled);
    ui->textBrowser->setEnabled(enabled);
    ui->menu_2->setEnabled(enabled);
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


    }
    else
        return;
}


