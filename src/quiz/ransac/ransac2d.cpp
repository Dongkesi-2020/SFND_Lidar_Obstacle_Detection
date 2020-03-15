/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	double bestA = 0;
	double bestB = 0;
	double bestC = 0;
	double bestN = 0;
	int maxNumberOfInliers = 0;
	// For max iterations 
	for (int i = 0; i < maxIterations; ++i) {
	// Randomly sample subset and fit line
		int indexP = rand() % cloud->points.size();
		int indexQ = rand() % cloud->points.size();
		while (indexP == indexQ) {
			indexP = rand() % cloud->points.size();
		}
		cout << "index_p: " << indexP << endl;
		cout << "index_q: " << indexQ << endl;

		double x1 = cloud->points[indexP].x;
		double y1 = cloud->points[indexP].y;
		double x2 = cloud->points[indexQ].x;
		double y2 = cloud->points[indexQ].y;

		double A = y1 - y2;
		double B = x2 - x1;
		double C = x1 * y2 - x2 * y1;
		double N = sqrt(A*A + B*B);
	// Measure distance between every point and fitted line
		int numberOfInliers = 0;
		for (auto point : cloud->points) {
			double x = point.x;
			double y = point.y;

			double d = abs(A * x + B * y + C) / N;
	// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol) {
				numberOfInliers++;
			}
		}
		if (numberOfInliers > maxNumberOfInliers) {
			maxNumberOfInliers = numberOfInliers;
			bestA = A;
			bestB = B;
			bestC = C;
			bestN = N;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	for (auto i = 0; i != cloud->points.size(); ++i) {
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;

		double d = abs(bestA * x + bestB * y + bestC) / bestN;
		if (d < distanceTol) {
			inliersResult.insert(i);
		}
	}

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for (int i = 0; i < maxIterations; ++i) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand() % (cloud->points.size()));
		}

		double x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto iter = inliers.begin();
		pcl::PointXYZ point = cloud->points[*iter++];
		x1 = point.x;
		y1 = point.y;
		z1 = point.z;
		point = cloud->points[*iter++];
		x2 = point.x;
		y2 = point.y;
		z2 = point.z;
		point = cloud->points[*iter];
		x3 = point.x;
		y3 = point.y;
		z3 = point.z;

		double A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		double B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		double C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		double D = -(A * x1 + B * y1 + C * z1);
		double N = sqrt(A * A + B * B + C * C);

		for (int index = 0; index < cloud->points.size(); ++index) {
			if (inliers.count(index) > 0)
				continue;
			point = cloud->points[index];
			double x = point.x;
			double y = point.y;
			double z = point.z;
			double d = abs(A * x + B * y + C * z + D) / N;

			if (d <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
