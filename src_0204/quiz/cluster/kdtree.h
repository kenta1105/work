/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{
		// 処理なし
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL) // ノードが空の時
		{
			*node = new Node(point, id);
		}
		else
		{
			int axis = depth%2; // 分割の軸
			//int axis = point.size(); // 何次元でも対応可
			if (point[axis] < ((*node)->point[axis]))
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		insertHelper(&root, 0, point, id)
		// while ver
		if (root == NULL)
		{
			root = new Node(point, id);			
		}
		else
		{
			Node** node = &root;
			int ptSize  = point.size();
			int depth   = 0;
			while (*node != NULL)
			{
				int axis = depth % ptSize;
				if (point[axis] < ((*node)->point[axis]))
				{
					node = &((*node)->left);
				}
				else
				{
					node = &((*node)->right);
				}
				depth++;
			}
			*node = new Node(point,id);
		}
	}

	void searchHelper(Node* node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if (node == NULL)
		{
			return;
		}
		else
		{
			// 四角形に入ってるかどうかのチェック
			bool inBox = true;
			isInBox = (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol) && node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol))
			// // 何次元でも対応可能
			// for (int i=0; i<target.size(); i++)
			// {
			// 	inBox &= (std::abs(node->point[i] - target[i]) <= distanceTol); 
			// }
			// 円内に入ってるかのチェック and クラスターにidの追加
			if (isInBox)
			{
				float dist = std::sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
				if (dist <= distanceTol)
				{
					ids.push_back(node->id);
				}
				// // 拡張ver
				// float dist = 0.0;
				// for (int i=0; i<target.size(); i++)
				// {
				// 	dist += ((node->point[i] - target[i]) * (node->point[i] - target[i]));
				// }
				// if (dist <= (distanceTol*distanceTol))
				// {
				// 	ids.push_back(node->id);
				// }
			}
			// 次のノードの探索
			int index = depth % target.size();
			if ((target[index]-distanceTol) < node->point[index])
			{
				searchHelper(node->left, depth+1, target, distanceTol, ids);
			}
			if ((target[index] + distanceTol) > node->point[index])
			{
				searchHelper(node->right, depth+1, target, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};




