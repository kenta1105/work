#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 
#include <Eigen/Core> 
#include <utility>
#include <cmath>
#include <pcl/common/common.h>

// 物標の向きも考慮したボックス
struct BoxQ
{
	Eigen::Vector3f bboxTransform;     // 重心
	Eigen::Quaternionf bboxQuaternion; // 向き(クオータニオン)
	float cube_length;                 // ボックスのx軸方向の長さ
    float cube_width;                  // ボックスのy軸方向の長さ
    float cube_height;                 // ボックスのz軸方向の長さ
};

// 物標の向きを考慮しないボックス
struct Box
{
	float x_min; // ボックス内の点の最小x座標
	float y_min; // ボックス内の点の最小y座標
	float z_min; // ボックス内の点の最小z座標
	float x_max; // ボックス内の点の最大x座標
	float y_max; // ボックス内の点の最大y座標
	float z_max; // ボックス内の点の最大z座標
};

// L-Shape Fitting
class Fitting
{
	private:
		const float dtheta = 1.0f * (M_PI/180.0f); // 角度分解能[deg]
	
	public:
		Fitting();  // コンストラクタ
		~Fitting(); // デストラクタ
    	BoxQ fitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud); // ボックス化
	
	private:
		void  searchRectangle (Eigen::MatrixXf& cluster_2d, BoxQ& box);                   // 最適な矩形の探索
    	float areaCriterion(Eigen::MatrixXf& cluster_2d_vc);                              // 面積最小化の評価関数
    	void  getBoundingBox(float optimalTheta, Eigen::MatrixXf& cluster_3d, BOxQ& box); // バウンディングボックスの作成
};


#endif