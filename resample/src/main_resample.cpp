#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>

int main()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile("../../dataset/bunny_cloud_downsamp.pcd", *cloud);

    // Lets introduce noise onto the point cloud
	srand((unsigned)time(NULL));
	int num_points = cloud->size();
	for(int i=0; i<num_points/2; i++)
	{
		int idx = (rand() % num_points) + 1;
		cloud->points[idx].x += 0.01 * (float)rand() / RAND_MAX;
		cloud->points[idx].y += 0.01 * (float)rand() / RAND_MAX;
		cloud->points[idx].z += 0.01 * (float)rand() / RAND_MAX;
	}

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(4);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(*mls_points);

    // Save output
    pcl::io::savePCDFile("../../dataset/mls_bunny_cloud.pcd", *mls_points);

    //---------------------------------------------
	//-------------Display Cloud VTK---------------
	//---------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Test Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(-0.1,-0.1, 0,   0, 0, 0,   0, 0, 1);
	viewer->setCameraFieldOfView(0.523599);
	viewer->setCameraClipDistances(0.00522511, 50); 

	// Clear the view
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

    // Create a viewport 1
    int v1(0);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText ("No Upsampling", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud1", v1);

    // Create a viewport 2
    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->addText ("Upsampling", 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointNormal> (mls_points, "cloud2", v2);

	// Rendering Properties
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

	// viewer->resetCamera ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}