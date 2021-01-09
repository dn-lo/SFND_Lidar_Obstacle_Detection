// Euclidean clustering implementation

#ifndef EUCLIDEANCLUSTER_H_
#define EUCLIDEANCLUSTER_H_

#include <pcl/point_cloud.h>
#include <chrono>
#include <vector>
#include <iostream>
// using KD-Tree structure (also include .cpp to help linker since you are including header)
#include "kdTree.h"
#include "kdTree.cpp"

template<typename PointT>
void proximity(int id, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& clusterIdx, KdTree* tree, float distanceTol, std::vector<bool> &visitedIds);

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);

#endif