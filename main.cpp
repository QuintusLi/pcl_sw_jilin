#include <QCoreApplication>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("C:/Users/23824/Desktop/rabbit.ply", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_ply.ply \n");
        system("PAUSE");
        return (-1);
    }
    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(cloud);
    system("PAUSE");
    return (0);
}
