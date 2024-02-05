#include "box.h"
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <pcl/common.h>

// コンストラクタ
Fitting:Fitting()
{
    // 処理なし
}

// デストラクタ
Fitting:~Fitting()
{
    // 処理なし
}

// ボックス化
BoxQ Fitting::fitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterClouds)
{
    BoxQ box; // ボックス情報
    Eigen::MatrixXf cluster_3d(3, clusterClouds->points.size()); // 行：3,列：点群の数の行列(各列の1,2,3行目にx,y,zを格納して、座標を行列で管理する)
    int row = 0; // 列

    // TODO::関数の処理を完成させてください。
    // 1. 行列に点群の座標を保存
    // 答え開始
    for (int i=0; i<clusterClouds->points.size(); i++)
    {
        cluster_3d(0, row) = clusterClouds->points[i].x;
        cluster_3d(1, row) = clusterClouds->points[i].y;
        cluster_3d(2, row) = clusterClouds->points[i].z;
        row++;
    }
    // 答え終了

    // 最適な矩形の探索
    searchRectangle(cluster_3d, box);

    return box;
}

// 最適な矩形の探索
void Fitting::searchRectangle(Eigen::MatrixXf& cluster_3d, BoxQ& box)
{
    float maxCost = -FLT_MAX; // 最大コスト
    float optimalTheta = 0.0; // 最適な角度(=最大コストの時の角度)

    // x,y,zの行列をx,yの行列に変換
    Eigen::MatrixXf cluster_2d = cluster_3d.topRows(2);      // 上２行(x,y)だけ抽出

    // TODO::以下の処理を実装し、関数を完成させてください。
    for (float theta = 0.0F; theta <= M_PI / 2.0F; theta += dtheta) // 90度まで探索
    {
        // 1. 回転行列の作成
        Eigen::Matrix2f transM;
        transM << cosf(theta), sinf(theta), -sinf(theta), cosf(theta); // 回転行列

        // 2. 座標変換
        Eigen::MatrixXf cluster_vc = transM * cluster_2d;  

        // 3. 評価関数の実行
        float cost = areaCriterion(cluster_vc);

        // 4. コスト, 最適角度の更新
        if (maxCost < cost)
        {
            maxCost = cost;
            optimalTheta = theta;
        }
    }

    // 最適な矩形の時のバウンディングボックスを取得
    getBoundingBox(optimalTheta, cluster_3d, box);
}

// 評価関数(面積最小化)
float Fitting::areaCriterion(Eigen::MatrixXf& cluster_2d_vc)
{
    float alfa = 0.0; // 評価コスト
    // TODO::処理を実装してください。
    // 答え開始
    float c1_max = cluster_2d_vc.row(0).maxCoeff();
    float c2_max = cluster_2d_vc.row(1).maxCoeff();
    float c1_min = cluster_2d_vc.row(0).minCoeff();
    float c2_min = cluster_2d_vc.row(1).minCoeff();

    alfa = -1*(c1_max - c1_min)*(c2_max - c2_min);
    // 答え終了
    return alfa;
}

// バウンディングボックスの取得
void Fitting::getBoundingBox(float optimalTheta, Eigen::MatrixXf& cluster_3d, BOxQ& box)
{
    // x,y,zの行列をx,yの行列に変換
    Eigen::MatrixXf cluster_2d = cluster_3d.topRows(2);      // 上２行(x,y)だけ抽出
    // TODO::関数を実装してください。
    // 答え開始
    // 1. 回転行列の作成
    Eigen::Matrix2f minTrans;
    minTrans << cosf(optimalTheta), sinf(optimalTheta), -sinf(optimalTheta), cosf(optimalTheta); // 回転行列

    // 2. 回転後の座標に変換
    Eigen::MatrixXf cluster_final = minTrans * cluster_2d;

    // 3. 回転後の座標でのx,y,zの最大値/最小値を取得
    float c1_max = cluster_final.row(0).maxCoeff();
    float c2_max = cluster_final.row(1).maxCoeff();
    float c1_min = cluster_final.row(0).minCoeff();
    float c2_min = cluster_final.row(1).minCoeff();
    float c3_max = cluster_3d.row(2).maxCoeff();
    float c3_min = cluster_3d.row(2).minCoeff();

    // 4. 回転後の座標での重心座標を算出
    Eigen::Vector2f centerPoint;
    centerPoint << (c1_max + c1_min)/2.0f, (c2_max + c2_min)/2.0f; // 回転した後の座標=移した座標

    // 5. 重心座標を元の座標系に変換
    centerPoint = minTrans.inverse() * centerPoint; // 元の座標に戻す

    // 6. ボックス情報(重心位置/長さ/幅/高さ/向き)を保存
    box.bboxtransform << centerPoint(0), centerPoint(1), (c3_max + c3_min) / 2.0f;
    box.cube_length = c1_max - c1_min;
    box.cube_width  = c2_max - c2_min;
    box_cube.height = c3_max - c3_min;
    Eigen::Vector3f axis; // 回転の軸
    axis << 0, 0, 1;      // Z軸を回転の軸に設定
    box.bboxQuaternion = Eigen::AngleAxisf(optimalTheta, axis);
    // 答え終了
}


