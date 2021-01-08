// RANSAC plane fitting

#ifndef RANSACPLANE_H_
#define RANSACPLANE_H_

#include <pcl/point_cloud.h>
#include <unordered_set>
#include <iostream>
#include <chrono>

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif