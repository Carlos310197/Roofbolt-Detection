#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <valarray>
#include <unordered_set>

#define NUM_NEIGHBORS 10

void remove(std::vector<int> &v)
{
	std::vector<int>::iterator itr = v.begin();
	std::unordered_set<int> s;

	for (auto curr = v.begin(); curr != v.end(); ++curr)
	{
		if (s.insert(*curr).second)
		{
			*itr++ = *curr;
		}
	}

	v.erase(itr, v.end());
}

int main()
{
	// Create pointer to point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Read point cloud data
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../dataset/bunny_cloud.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " points" << std::endl;

	// Lets introduce noise onto the point cloud
	srand((unsigned)time(NULL));
	int num_points = cloud->size();
	for(int i=0; i<num_points/10000; i++)
	{
		int idx = (rand() % num_points) + 1;
		cloud->points[idx].x += 0.02 * (float)rand() / RAND_MAX;
		cloud->points[idx].y += 0.02 * (float)rand() / RAND_MAX;
		cloud->points[idx].z += 0.02 * (float)rand() / RAND_MAX;
	}

	// Create kdtree object
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;

	// K nearest neighbor search
	int K = NUM_NEIGHBORS + 1; // number of neighboors

	// kdtree searching vectors
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);

	// KNN points
	pcl::PointCloud<pcl::PointXYZ>::Ptr KNNpoints(new pcl::PointCloud<pcl::PointXYZ>);
	KNNpoints->resize(K);
	std::vector<float> pointKNNDistance_vec;

	// Fitting plane vectors
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg; // segmentation object
	std::vector<int> inliers_idx;

	// start timer
	auto start = std::chrono::steady_clock::now();

	// Search KNN points
	int cont_failed = 0;
	int cont = 0;
	for (const auto &point : *cloud)
	{
		if (kdtree.nearestKSearch(point, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			// Find distances to the KNN points
			std::valarray<float> pointKNNDistance(pointKNNSquaredDistance.data(), K);
			pointKNNDistance = sqrt(pointKNNDistance);
			pointKNNDistance_vec.assign(std::begin(pointKNNDistance), std::end(pointKNNDistance));
			// for (int i = 1; i < pointIdxKNNSearch.size(); i++) std::cout << "Distance: " << pointKNNDistance_vec[i] << std::endl;

			// Compute mean and standard deviation
			double sum = std::accumulate(pointKNNDistance_vec.begin(), pointKNNDistance_vec.end(), 0.0);
			double mean = sum / (pointKNNDistance_vec.size() - 1);
			double sq_sum = std::inner_product(pointKNNDistance_vec.begin(), pointKNNDistance_vec.end(), pointKNNDistance_vec.begin(), 0.0);
			double stdev = std::sqrt(sq_sum / (pointKNNDistance_vec.size() - 1) - mean * mean);
			// std::cout << "mean: " << mean << ", 2*std: " << 2*stdev << std::endl;

			// Get the KNN points
			for (int i = 0; i < pointIdxKNNSearch.size(); i++)
				KNNpoints->points[i] = cloud->points[pointIdxKNNSearch[i]];

			///// Fit a plane to the neighborhood
			seg.setOptimizeCoefficients(true);// Optional
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE); // fit to a plane
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(2*stdev);  // minimum distance of 2*stdev
			seg.setInputCloud(KNNpoints);		  // Neighborhood as the point cloud
			seg.segment(*inliers, *coefficients); // get inliers and plane coefficients

			for (const auto &idx : inliers->indices)
			{
				// std::cout << "idx: " << idx << std::endl;
				// std::cout << "Storing index: " << pointIdxKNNSearch[idx] << std::endl;
				inliers_idx.push_back(pointIdxKNNSearch[idx]);
			}

			cont++;

			if (inliers->indices.size() == 0)
			{
				cont_failed++;
				std::cout << "Fail!" << std::endl;
				continue;
			}
		}
	}
	// Remove duplicates from the inliers_idx vector
	remove(inliers_idx);
	std::cout << "Number of inliers: " << inliers_idx.size() << std::endl;
	std::cout << "Number of outliers: " << cloud->points.size() - inliers_idx.size() << std::endl;

	// stop timer
	auto end = std::chrono::steady_clock::now();

	// Number of points that failed to fit a plane
	std::cout << "Number of points that fail to fit a plane: " << cont_failed << std::endl;
	std::cout << "Failed Percentage: " << (float)cont_failed * 100.0 / (cloud->points.size()) << "%" << std::endl;

	// Display measured time
	cout << "Filtering elapsed time in milliseconds: "
		 << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
		 << " ms" << endl;

	// Copy the original to the filtered cloud
	pcl::copyPointCloud(*cloud, *filt_cloud);
	for (int i = 0; i < filt_cloud->points.size(); i++)
	{
		filt_cloud->points[i].r = 255;
		filt_cloud->points[i].g = 0;
		filt_cloud->points[i].b = 0;
	}
	for (int i = 0; i < inliers_idx.size(); i++)
	{
		// add the color
		filt_cloud->points[inliers_idx[i]].r = 255;
		filt_cloud->points[inliers_idx[i]].g = 255;
		filt_cloud->points[inliers_idx[i]].b = 255;
	}

	//---------------------------------------------
	//-------------Display Cloud VTK---------------
	//---------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Test Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0.1, 0.1,    0, 0, 0,   0, 0, 1);
	viewer->setCameraFieldOfView(0.523599);
	viewer->setCameraClipDistances(0.00522511, 50); 

	// Clear the view
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

	// The point cloud
	viewer->addPointCloud<pcl::PointXYZRGB>(filt_cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

	// viewer->resetCamera ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}
