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


pcl::PointCloud<pcl::PointXYZI>::Ptr ProcessPointClouds<pcl::PointXYZI>::FilterCloud(pcl::PointCloud<PointXYZI>::Ptr cloud, float sampleResolution, crop cropArea)
{
    // ダウンサンプリング
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg; // ダウンサンプリングを実施するオブジェクトの作成
    vg.setInputCloud(cloud);           // 点群のセット
    vg.setLeafSize(sampleResolution, sampleResolution, sampleResolution); // ダウンサンプリング粒度を設定
    vg.filter(*sampledCloud); // ダウンサンプリング実行

    std::cout << sampledCloud->points.size() << "data points after downsampling" << std::endl;

    // 遠方の点群,自車の反射点群の削除
    pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // ToDo::遠方の点群、自車の反射点群を削除する処理を完成させてください。
    // 答え開始
    for (pcl::PointXYZI point : cropedCloud->points)
    {
        // 遠方の点の判定
        bool isIn_x  = ((cropArea.min_x <= point.x) & (point.x <= cropArea.max_x));
        bool isIn_y  = ((cropArea.min_y <= point.y) & (point.y <= cropArea.max_y));
        bool isIn_xy = isIn_x & isIn_y;

        // 自車に反射した点の判定
        bool isIn_ego = (std::abs(point.x) <= cropArea.ego_x) & (std::abs(point.y) <= cropArea.ego_y);

        // 範囲内の点を保存
        bool isIn = isIn_xy & ~isIn_ego;
        if (isIn)
        {
            croppedCloud->push_back(point);
        }
    }
    // 答え終了

    std::cout << croppedCloud->points.size() << "data points after cropping" << std::endl;

    return sampledCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // クラスターの配列

    // TODO:: 関数を実装して下さい
    // 1. KD-Treeの設定
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); // 木の設定
    tree->setInputCloud (cloud);                                             // 点群の設定

    // 2. ユークリッドクラスタリングの実行
    std::vector<pcl::PointIndices> cluster_indices;                          // クラスターのインデックス
    pcl::EuclideanClusterExtraction<PointT> ec;                              // ユークリディアンクラスターオブジェクト
    ec.setClusterTolerance (clusterTolerance);                               // 許容値
    ec.setMinClusterSize (minSize);                                          // クラスターの最小点群個数の設定
    ec.setMaxClusterSize (maxSize);                                          // クラスターの最大点群個数の設定
    ec.setSearchMethod (tree);                                               // 探索方法にKD-Treeを設定
    ec.setInputCloud (cloud_filtered);                                       // 点群を設定
    ec.extract (cluster_indices);                                            // クラスターのインデックスを取得

    // 3. クラスターごとに分類
    for (pcl::PointIndices getIndices : cluster_indices)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>); 
        for (int index : getIndices.indices)
        {
            cloud_cluster.points.push_back(cloud->points[index]);
        }
        cloud_cluster->width    = cloud_cluster->size ();
        cloud_cluster->height   = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
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

// 点群の読み込み
pcl::PointCloud<pcl::PointXYZI>::Ptr ProcessPointClouds<pcl::PointXYZI>::loadPcd(std::string file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //fileから点群データの読み込み
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cout << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

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