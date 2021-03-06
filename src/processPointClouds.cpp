// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

#include <unordered_set>

#include "kdtree.h"

#define PROJECT
// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering Create the filtering object
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>);

  typename pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*cloud_filtered);

  typename pcl::PointCloud<PointT>::Ptr cloud_cropped(
      new pcl::PointCloud<PointT>);
  typename pcl::CropBox<PointT> cropBox;
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);
  cropBox.setInputCloud(cloud_filtered);
  cropBox.filter(*cloud_cropped);

  // optional remove ego car roof points
  std::vector<int> indices;
  pcl::CropBox<PointT> cropRoof;
  cropRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  cropRoof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  cropRoof.setInputCloud(cloud_cropped);
  cropRoof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_cropped);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_cropped);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud_cropped;
}

#ifdef PROJECT
template <typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  size_t pointSize = cloud->points.size();
  for (int i = 0; i < maxIterations; ++i) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 3) {
      inliers.insert(rand() % (pointSize));
    }

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto iter = inliers.begin();
    PointT point = cloud->points[*iter++];
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

    float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float D = -(A * x1 + B * y1 + C * z1);
    float N = sqrt(A * A + B * B + C * C);

    for (int index = 0; index < pointSize; ++index) {
      if (inliers.count(index) > 0) continue;
      point = cloud->points[index];
      float x = point.x;
      float y = point.y;
      float z = point.z;
      float d = abs(A * x + B * y + C * z + D) / N;

      if (d <= distanceTol) inliers.insert(index);
    }

    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliers =
      RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

  typename pcl::PointCloud<PointT>::Ptr cloud_obstacles(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloud_plane(
      new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points[index];
    if (inliers.count(index))
      cloud_plane->points.push_back(point);
    else
      cloud_obstacles->points.push_back(point);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(cloud_obstacles, cloud_plane);
  return segResult;
}
#else

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr cloud_obstacles(
      new pcl::PointCloud<PointT>),
      cloud_plane(new pcl::PointCloud<PointT>);
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_plane);

  // Create the filtering object
  extract.setNegative(true);
  extract.filter(*cloud_obstacles);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(cloud_obstacles, cloud_plane);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // TODO:: Fill in this function to find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}
#endif

#ifdef PROJECT

template <typename PointT>
void proximity(std::vector<bool>& processedPointsIdx, std::vector<int>& cluster,
               int idx, const typename pcl::PointCloud<PointT>::Ptr cloud,
               KdTree* tree, float distanceTol) {
  if (processedPointsIdx[idx]) {
    return;
  } else {
    processedPointsIdx[idx] = true;
    cluster.push_back(idx);
    PointT& point = cloud->points[idx];
    std::vector<float> target{point.x, point.y, point.z};
    std::vector<int> nearbyPoints = tree->search(target, distanceTol);
    for (auto i : nearbyPoints) {
      proximity<PointT>(processedPointsIdx, cluster, i, cloud, tree,
                        distanceTol);
    }
  }
}

template <typename PointT>
std::vector<std::vector<int>> euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree,
    float distanceTol) {
  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::vector<int>> clusters;

  size_t pointsSize = cloud->points.size();
  std::vector<bool> processedPointsIdx(pointsSize, false);
  for (size_t i = 0; i != pointsSize; ++i) {
    if (processedPointsIdx[i]) {
      continue;
    }
    std::vector<int> cluster;
    proximity<PointT>(processedPointsIdx, cluster, i, cloud, tree, distanceTol);
    clusters.push_back(cluster);
  }

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  KdTree* tree = new KdTree;

  for (int i = 0; i < cloud->points.size(); i++) {
    PointT& point = cloud->points[i];
    std::vector<float>* v = new std::vector<float>();
    v->push_back(point.x);
    v->push_back(point.y);
    v->push_back(point.z);
    tree->insert(*v, i);
  }

  std::vector<std::vector<int>> cluster_indices =
      euclideanCluster<PointT>(cloud, tree, clusterTolerance);

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    if (it->size() > maxSize || it->size() < minSize) {
      continue;
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(
        new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->begin(); pit != it->end();
         ++pit) {
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}
#else
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles Creating the KdTree object for the search method of the
  // extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(
        new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}
#endif

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}