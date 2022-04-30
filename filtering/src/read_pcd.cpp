#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../dataset/scan_map.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << std::endl;

    //---------------------------------------------
    //-------------Display Cloud VTK---------------
    //---------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Test Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    // Clear the view
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();

    // The point cloud
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // viewer->resetCamera ();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return (0);
}