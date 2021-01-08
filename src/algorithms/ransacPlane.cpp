// RANSAC plane fitting

#include "ransacPlane.h"

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Separated obstacle and plane PCD
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>()), planeCloud(new pcl::PointCloud<PointT>());
	// Plane points indices
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	for (int i=0; i<maxIterations; i++) {
		// Allocate current inlier set
		std::unordered_set<int> inliers;
		// Randomly select two indices. Notice:
		// Unordered set has unique elements therefore it will have no repetitions
		while (inliers.size() < 3) {
			inliers.insert( rand() % cloud->points.size() );
		}
		// Sampled points
		PointT p1, p2, p3;
		auto itr = inliers.begin();
		p1 = cloud->points[*itr];
		itr++;
		p2 = cloud->points[*itr];
		itr++;
		p3 = cloud->points[*itr];
		float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
		float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
		float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
		float D = - (A*p1.x + B*p1.y + C*p1.z);
		// For all points in cloud
		for (int idx = 0; idx < cloud->points.size(); idx++) {
			// Skip if point was already in result
			if (inliers.count(idx)>0) 
				continue;
			// Measure distance between every point and fitted line
			PointT p = cloud->points[idx];
			float distance = fabs(A*p.x + B*p.y + C*p.z + D) / sqrt(A*A + B*B + C*C);
			// If distance is lower than threshold insert index as inlier
			if (distance <= distanceTol) {
				inliers.insert(idx);
			}
		}
		// If number of inliers is best one, set to result
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Separate plane and obstacle cloud
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}
	// Return PCD pair
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}