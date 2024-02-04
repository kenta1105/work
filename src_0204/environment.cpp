/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// 
void initCamera(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // viewerの初期化
    viewer->setBackgroundColor (0, 0, 0); // 背景色を黒に設定
    viewer->initCameraParameters();       // カメラの位置/見る方向を初期化
    viewer->addCoordinateSystem (1.0);    // 座標軸の設定
    
    // カメラの位置(どの方向から見るか)の設定
    int distance = 15;                    // カメラの設置位置[m]
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); // カメラの位置/見る方向を設定
}

// 点群認識処理
void processPointClouds(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // 点群の読み込み
    processPointClouds<pcl::PointXYZI> pointProcessor; // 点群認識処理のオブジェクト
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd"); // PCD(点群)ファイルの読み込み
    renderPointCloud(viewer,inputCloud,"inputCloud"); // 読み込んだ点群データの表示

    // 点群のフィルタリング(ダウンサンプリング + 遠方の点群/自車に反射した点群の削除)
    float sampleResolution = 0.2;               // ダウンサンプリング粒度(0.2[m])
    crop cropArea = (-20, 25, -6, 6.5, -3, 3, 3, 1.7)  // 削除するエリア[m]
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor.filterCloud(inputCloud, sampleResolution, cropArea); // 点群の削除
    renderPointCloud(viewer, filteredCloud, "filteredCloud"); // フィルタリングした点群データの表示




}


int main ()
{
    std::cout << "starting process point cloud" << std::endl;

    // viewerの設定
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); // viewerオブジェクトの作成
    initCamera(viewer); // カメラ(viewerの見え方の設定)
    
    // 点群認識処理
    processPoint(viewer);         // 点群認識処理

    // viewerをずっと起動させる
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}