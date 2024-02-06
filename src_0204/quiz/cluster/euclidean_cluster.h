#ifndef __EUCLIDEAN_CLUSTER_H__
#define __EUCLIDEAN_CLUSTER_H__
#include "kdtree.h"

// ユークリッドクラスタリング
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceThreshold);

// Helper関数
void clusterHelper(const std::vector<float>>& points, const int id, KdTree* tree, std::vector<int>& cluster, std::vector<bool>& checked, float distanceThreshold)

#endif