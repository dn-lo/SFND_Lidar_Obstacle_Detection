// Process 3d highway enviroment using PCL and implemented functions
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
// using implemented algorithms for RANSAC and clustering (also include .cpp to help linker since you are including header)
#include "algorithms/ransacPlane.h"
#include "algorithms/ransacPlane.cpp"
#include "algorithms/euclideanCluster.h"
#include "algorithms/euclideanCluster.cpp"


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // Filter input PCD to downsample it on a grid and select a box region of interest
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(pointCloud, 0.19, Eigen::Vector4f(-10., -5, -3., 1), Eigen::Vector4f(30., +6.5, 3., 1));

    // Segment point cloud using RANSAC - PCL implementation
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(filterCloud, 100, 0.2);
    
    // Segment point cloud using RANSAC - my implementation ransacPlane
    auto segmentCloud = ransacPlane<pcl::PointXYZI>(filterCloud, 100, 0.13);
 
    // Render obstacle point cloud using red and road using green
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacle point cloud - PCL implementation
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.45, 9, 500);

    // Cluster obstacle point cloud - my implementation euclideanCluster
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = euclideanCluster<pcl::PointXYZI>(segmentCloud.first, 0.435, 14, 1000);
    
    // Use rgb colors
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // Render each cluster with a different color    
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        // Also render bounding box
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // Create point processor with x, y, z and intensity
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    // You tell streamPcd a directory that contains all the sequentially ordered pcd files you want to process, 
    // and it returns a chronologically ordered vector of all those file names, called stream. 
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    // Create PCD stream iterator and point cloud
    auto streamItr = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;

    // PCL viewer update loop
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Render plain PCD
        // renderPointCloud(viewer, pointCloud, "pointCloud");

        // Load pcd and run obstacle detection process
        pointCloud = pointProcessorI.loadPcd((*streamItr).string());
        cityBlock(viewer, pointProcessorI, pointCloud);


        // Go to next PCD file
        streamItr++;
        if(streamItr == stream.end())
            streamItr = stream.begin();

        // Used to modify update rate
        viewer->spinOnce(200);
    } 
}