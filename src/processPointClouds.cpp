// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes
    , Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint
    , Eigen::Vector4f roofMin, Eigen::Vector4f roofMax)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in the function to do voxel grid point reduction and region based filtering

    // 1. Apply Voxel Grid Downsampling
    pcl::VoxelGrid<PointT> voxelGrid;
    auto voxelFiltered = std::make_shared<pcl::PointCloud<PointT>>();

    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);  // cube resolution
    voxelGrid.filter(*voxelFiltered);

    std::cout << "Voxel grid filter - cloud size: " << voxelFiltered->size() << std::endl;

    // 2. Region of Interest Filtering (CropBox)
    pcl::CropBox<PointT> region(true);  // since dealing with points inside that CropBox
    region.setMin(minPoint);            
    region.setMax(maxPoint);    

    auto cloudRegion = std::make_shared<pcl::PointCloud<PointT>>();
    region.setInputCloud(voxelFiltered);
    region.filter(*cloudRegion);

    std::cout << "ROI filter - cloud size: " << cloudRegion->size() << std::endl;

    // 3. Remove points inside a "roof" region
    // indices inside the region
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(roofMin);
    roof.setMax(roofMax);
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);    // indices of points to REMOVE

    auto inliers = std::make_shared<pcl::PointIndices>();
    for (int point_index : indices)
    {
        inliers->indices.push_back(point_index);
    }

    // REMOVE those inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);        // true to REMOVE these points
    extract.filter(*cloudRegion);     // update cloudRegion

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    auto cloud_p = std::make_shared<pcl::PointCloud<PointT>>();
    auto cloud_obs = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // subtract plane clouds from input cloud
    extract.setNegative(false);
    extract.filter(*cloud_p);

    // extract obtacles clouds from input cloud
    extract.setNegative(true);
    extract.filter(*cloud_obs);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obs, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in this function to find inliers for the cloud.
    auto inliers = std::make_shared<pcl::PointIndices>();
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    auto segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
    for(const auto& getIndices : clusterIndices)
    {
        auto cloudCluster = std::make_shared<pcl::PointCloud<PointT>>();
        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    auto cloud = std::make_shared<pcl::PointCloud<PointT>>();

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud
    , int maxIterations
    , float distanceTol)
{
    std::unordered_set<int> bestInliersResult;
    srand(time(NULL));

    const int cloudSize = cloud->points.size();

    for (int i = 0; i < maxIterations; ++i)
    {
        std::unordered_set<int> inliers;

        // Randomly sample 3 unique points
        while (inliers.size() < 3)
            inliers.insert(rand() % cloudSize);

        auto it = inliers.begin();
        const PointT& p1 = cloud->points[*it]; ++it;
        const PointT& p2 = cloud->points[*it]; ++it;
        const PointT& p3 = cloud->points[*it];

        // Plane coefficients from cross product
        float v1x = p2.x - p1.x;
        float v1y = p2.y - p1.y;
        float v1z = p2.z - p1.z;

        float v2x = p3.x - p1.x;
        float v2y = p3.y - p1.y;
        float v2z = p3.z - p1.z;

        float A = v1y * v2z - v1z * v2y;
        float B = v1z * v2x - v1x * v2z;
        float C = v1x * v2y - v1y * v2x;

        float norm = std::sqrt(A * A + B * B + C * C);
        if (norm == 0.0f)
            continue;

        float invNorm = 1.0f / norm;
        float D = -(A * p1.x + B * p1.y + C * p1.z);

        // Measure distance for all points
        for (int j = 0; j < cloudSize; ++j)
        {
            const PointT& pt = cloud->points[j];

            float distance =
                std::fabs(A * pt.x + B * pt.y + C * pt.z + D) * invNorm;

            if (distance <= distanceTol)
                inliers.insert(j);
        }

        if (inliers.size() > bestInliersResult.size())
            bestInliersResult = std::move(inliers);
    }

    return bestInliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::customSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    // --- Call your custom RANSAC PLANE algorithm ---
    std::unordered_set<int> inliersSet = RansacPlane(cloud, maxIterations, distanceThreshold);

    if (inliersSet.size() == 0)
    {
        std::cerr << "No plane found." << std::endl;
        return {};
    }

    // --- Convert unordered_set → pcl::PointIndices ---
    auto inliers = std::make_shared<pcl::PointIndices>();
    for (int idx : inliersSet)
        inliers->indices.push_back(idx);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // --- Separate the cloud into plane & obstacles ---
    auto result = SeparateClouds(inliers, cloud);
    return result;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& clusters, std::vector<bool>& processed, std::shared_ptr<KdTree> tree, float distanceTol)
{
    processed[indice] = true;
    clusters.push_back(indice);
    auto nearest = tree->search(points[indice], distanceTol);
    for(int id : nearest)
    {
        if(!processed[id])
            clusterHelper(id, points, clusters, processed, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, std::shared_ptr<KdTree> tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);
    int pointIndex = 0;
    for(int pointIndex = 0; pointIndex < points.size(); pointIndex++)
    {
        if(processed[pointIndex])
        {
            continue;
        }
        std::vector<int> cluster;
        clusterHelper(pointIndex, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
    }
 
    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::customClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    // 1. Convert PCL -> vector<float>
    std::vector<std::vector<float>> points;
    points.reserve(cloud->points.size());

    for (auto& p : cloud->points)
        points.push_back({ p.x, p.y, p.z });

    // 2. Build KD-Tree
    auto tree = std::make_shared<KdTree>();
    tree->build(points);

    // 3. Perform custom clustering
    std::vector<std::vector<int>> clusterIndices =
        euclideanCluster(points, tree, clusterTolerance);

    // 4. Convert indices -> PCL clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (const auto& indices : clusterIndices)
    {
        if (indices.size() < minSize || indices.size() > maxSize)
            continue;

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());

        for (int idx : indices)
            cluster->points.push_back(cloud->points[idx]);

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "clustering took " << elapsedTime.count()
        << " milliseconds and found " << clusters.size()
        << " clusters" << std::endl;

    return clusters;
}