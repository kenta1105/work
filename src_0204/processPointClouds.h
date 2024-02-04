// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

// 遠方の点群/自車の反射点群を削除する範囲
struct Crop
{
    float min_x, max_x, min_y, max_y, min_z, max_z, ego_x, ego_y;

    Crop(float setMin_x, float setMax_x, float setMin_y, float setMax_y, float setMin_z, float setMax_z, float setEgo_x, float setEgo_y)
    : min_x(setMin_x), max_x(setMax_x), min_y(setMin_y), max_y(setMax_y), min_z(setMin_z), max_z(setMax_z), ego_x(setEgo_x), ego_y(setEgo_y)
    {
        // 処理なし
    }
};

// 点群認識処理オブジェクト
class ProcessPointClouds {
public:

    // コンストラクタ
    ProcessPointClouds();
    // デストラクタ
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    // 点群の読み込み
    pcl::PointCloud<PointXYZI>::Ptr loadPcd(std::string file);

    // フィルタリング(ダウンサンプリング + 遠方の点群/自車の反射点群の削除)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes, Crop cropArea);

    std::pair<pcl::PointCloud<PointXYZI>::Ptr, pcl::PointCloud<PointXYZI>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<PointXYZI>::Ptr cloud);

    std::pair<pcl::PointCloud<PointXYZI>::Ptr, pcl::PointCloud<PointXYZI>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */