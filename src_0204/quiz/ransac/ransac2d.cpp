/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// インライア点群(直線の点群)
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// アウトライア点群(外れ値の点群)
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::pair<pcl::PointCloud<PointXYZ>::Ptr, pcl::PointCloud<PointXYZ>::Ptr> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult; // 最終的なインライア(地面点群)
	std::srand(std::time(NULL));           // 乱数の種の設定
	
	// TODO: 関数を実装して下さい
	while (maxIterations--) // 反復回数だけ繰り返す
	{
		// 1. ランダムな2点の取得
		std::unordered_set<int> inliers;
		while (inliers.size() < 2)
		{
			inliers.insert(std::rand() % cloud->points.size());
		}
		// 2. 点の座標の取得
		std::unordered_set<int>::iterator iter = inlier.begin();
		const pcl::PointXYZ& pt1 = cloud->points[*iter++];
		const pcl::PointXYZ& pt2 = cloud->points[*iter];
		
		// float x1,y1,x2,y2;
		// x1 = cloud->points[*iter].x;
		// y1 = cloud->points[*iter].y;
		// iter++;
		// x2 = cloud->points[*iter].x;
		// y2 = cloud->points[*iter].y;
		// 3. 直線の式の定義
		// float A  = y1 - y2;
		// float B  = x2 - x1;
		// float C  = x1*y2 - x2*y1;

		// 3. 直線の式の定義
		float A  = pt1.y - pt2.y;
		float B  = pt2.x - pt1.x;
		float C  = pt1.x*pt2.y2 - pt2.x*pt1.y;
		// 4. 残りの点群が地面点群に含まれるかを点と直線の距離を用いて判断
		for (int index=0; index<cloud->points.size(); index++)
		{
			// すでにインライアに点群のインデックスが含まれてる場合、スキップする
			if (inliers.count(index) > 0)
			{
				continue;
			}
			// 点群の座標
			const pcl::PointXYZ& pt = cloud->points[*index];
			// float x = cloud->points[index].x;
			// float y = cloud->points[index].y;
			// 点と直線の距離の算出 
			// float d = std::fabs(A*x + B*y + C) / std::sqrt(A*A + B*B);
			float d = std::fabs(A*pt.x + B*pt.y + C) / std::sqrt(A*A + B*B);
			// dが距離閾値(distanceTol)以下の時、点群のインデックスを保存
			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}
		// 5. 最終的な地面点群を更新
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	} 

	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult();

	return segResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> planeResult; // 直線に分類する点群のインデックス
	std::srand(std::time(NULL));         // 乱数の種の設定
	
	// TODO: 関数を実装して下さい
	// 1. 地面点群のインデックスを抽出
	while (maxIterations--) // 反復回数だけ繰り返す
	{
		// 1. インライア(地面点群)の宣言(地面点群のインデックスを保存する)
		std::unordered_set<int> inliers;
		// 2. ランダムな2点の取得
		while (inliers.size() < 3)
		{
			inliers.insert(std::rand() % cloud->points.size());
		}
		// 3. 点群の座標の設定
		std::unordered_set<int>::iterator iter = inlier.begin();
		// float x1,y1,z1,x2,y2,z3,x3,y3,z3;
		// x1 = cloud->points[*iter].x;
		// y1 = cloud->points[*iter].y;
		// z1 = cloud->points[*iter].z;
		// iter++;
		// x2 = cloud->points[*iter].x;
		// y2 = cloud->points[*iter].y;
		// z2 = cloud->points[*iter].z;
		// iter++;
		// x3 = cloud->points[*iter].x;
		// y3 = cloud->points[*iter].y;
		// z3 = cloud->points[*iter].z;
		const pcl::PointXYZ& pt1 = cloud->points[*iter++];
		const pcl::PointXYZ& pt2 = cloud->points[*iter++];
		const pcl::PointXYZ& pt3 = cloud->points[*iter];
		// 4. 直線の式の定義
		// float A  = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		// float B  = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		// float C  = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		// float D  = -1*(A*x1 + B*y1 + C*z1);
		float A  = (pt2.y-pt1.y)*(pt3.z-pt1.z) - (pt2.z-pt1.z)*(pt3.y-pt1.y);
		float B  = (pt2.z-pt1.z)*(pt3.x-pt1.x) - (pt2.x-pt1.x)*(pt3.z-pt1.z);
		float C  = (pt2.x-pt1.x)*(pt3.y-pt1.y) - (pt2.y-pt1.y)*(pt3.x-pt1.x);
		float D  = -1*(A*pt1.x + B*pt1.y + C*pt1.z);
		// 4. 残りの点群が地面点群に含まれるかを点と直線の距離を用いて判断
		for (int index=0; index<cloud->points.size(); index++)
		{
			// すでにインライアに点群のインデックスが含まれてる場合、スキップする
			if (inliers.count(index) > 0)
			{
				continue;
			}
			// 点群の座標
			// float x = cloud->points[index].x;
			// float y = cloud->points[index].y;
			// float z = cloud->points[index].z;
			const pcl::PointXYZ& pt = cloud->points[index];
			// // 点と直線の距離の算出 
			// float d = std::fabs(A*x + B*y + C*z + D) / std::sqrt(A*A +B*B + C*C);
			float d = std::fabs(A*pt.x + B*pt.y + C*pt.z + D) / std::sqrt(A*A +B*B + C*C);
			// dが距離閾値(distanceTol)以下の時、点群のインデックスを保存
			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}
		// 5. 最終的な地面点群を更新
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	} 

	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult();

	return segResult;
}

int main ()
{
	// viewerオブジェクトの設定
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// 点群データの作成
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	std::unordered_set<int> inliers = Ransac(cloud, 0, 0);
	// std::unordered_set<int> inliers = RansacPlane(cloud, 0, 0);

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
