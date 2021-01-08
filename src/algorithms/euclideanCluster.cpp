// Euclidean clustering implementation

#include "euclideanCluster.h"

// Find nearby points and add their indices to clusterIdx
template<typename PointT>
void proximity(int id, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& clusterIdx, KdTree* tree, float distanceTol, std::vector<bool> &visitedIds)
{
	// Mark point as processed
	visitedIds[id] = true;
    // Add point index to cluster
	clusterIdx.push_back(id);
    // Find nearby points indices using Search method in KdTree class
	std::vector<int> nearby = tree->search({cloud->points[id].x, cloud->points[id].y, cloud->points[id].z}, distanceTol);
    // Iterate through each nearby point index
	for (int id : nearby) {
		// If nearby point has not been processed
		if (!visitedIds[id]) {
			// Repeat process with nearby point
			proximity<PointT>(id, cloud, clusterIdx, tree, distanceTol, visitedIds);
		}
	}
}

// Separate input cloud into clusters using Euclidean clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
	// Start of clustering process
  	auto startTime = std::chrono::steady_clock::now();
  	
	// Create KD-Tree using points in input cloud
	KdTree* tree = new KdTree;
	int id = 0;
	for (auto point : cloud->points) {
		tree->insert({point.x, point.y, point.z}, id);
		id++;
	}

	// Create vector of indices for each cluster
	std::vector<std::vector<int>> clusterIndices;

	// Create set of visited point ids 
	std::vector<bool> visitedIds(cloud->points.size(), false);
	
	// Iterate through points
	for (int id = 0; id < cloud->points.size(); id++) {
		// If point has not been processed
		if (!visitedIds[id]) {
			// Create a vector that will contain indices of the new cluster associated with the point
			std::vector<int> clusterIdx;
			// Add points closer than distance tolerance to clusterIdx (passed by reference)
			// Use vector of visited ids to avoid processing the same point twice
			proximity<PointT>(id, cloud, clusterIdx, tree, distanceTol, visitedIds);

			// Add new cluster only if its size is smaller than maxSize and larger than minSize
			if ((clusterIdx.size() >= minSize) && (clusterIdx.size() <= maxSize))
				clusterIndices.push_back(clusterIdx);
		}
	}

	// Output clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	
	// For each vector of cluster indices
	for(std::vector<int> clusterIdx : clusterIndices)
  	{
		// Create cluster and insert it into output clusters
  		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
  		for(int idx : clusterIdx)
            cluster->points.push_back(cloud->points[idx]);
		clusters.push_back(cluster);
  	}

	// End of clustering process
	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
	  
	return clusters;
}