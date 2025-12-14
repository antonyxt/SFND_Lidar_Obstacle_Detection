/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    auto lidar = new Lidar(cars, 0);
    auto cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "lidarPC");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    auto segmentCloud = pointProcessor.SegmentPlane(cloud, 100, 0.2);
    
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCLoud", Color(0,1,0));

    auto cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    int numColors = colors.size();
    for(const auto& cluster : cloudClusters)
    {
        std::cout << "Cluster Size:";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obtacle_" + std::to_string(clusterId), colors[clusterId % numColors]);
        auto box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
  
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //keep points in a 23×16×5 meter region
    Eigen::Vector4f minPoint(-8.0, -8.0, -2.0, 1.0);
    Eigen::Vector4f maxPoint(15.0, 8.0, 3.0, 1.0);

    //Remove Car Roof
    Eigen::Vector4f roofMin(-1.5, -1.7, -1.0, 1.0);
    Eigen::Vector4f roofMax(2.6, 1.7, -0.4, 1.0);
    // 1. Down sampling and filtering 
    auto filterCloud = pointProcessor->FilterCloud(inputCloud, 0.2 , minPoint, maxPoint, roofMin, roofMax);
    //renderPointCloud(viewer,filterCloud,"filterCloud");
    // 2. Segmenting the point cloud into obstacles and plane
    auto segmentCloud = pointProcessor->customSegmentPlane(filterCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "planeCLoud", Color(0,1,0));

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
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}