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
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

    void _insertInternal(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			// calculate axis
			uint ca = depth % 2;
			auto& nodeRef = *node;
			if(point[ca] < nodeRef->point[ca] )
			{
				_insertInternal(&nodeRef->left, depth+1, point, id);
			}
			else
			{
				_insertInternal(&nodeRef->right, depth+1, point, id);
			}
		}
		
	}
	void insert(std::vector<float> point, int id)
	{
		_insertInternal(&root, 0, point, id);
	}

    void _searchInternal(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node == NULL)
			return;

		if( ((node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol))) &&
		    ((node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol))) )
		{
			float xdiff = node->point[0] - target[0];
			float ydiff = node->point[1] - target[1];
			float distance = sqrt(xdiff*xdiff + ydiff*ydiff);

			if(distance <= distanceTol)
				ids.push_back(node->id);
		}
		uint ca = depth % 2;

		if(target[ca] - distanceTol < node->point[ca])
			_searchInternal(target, node->left, depth+1, distanceTol, ids);
		if(target[ca] + distanceTol > node->point[ca])
			_searchInternal(target, node->right, depth+1, distanceTol, ids);
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		_searchInternal(target, root, 0, distanceTol, ids);
		return ids;
	}
	
    // Build balanced tree
    Node* buildBalanced(std::vector<std::vector<float>>& points, std::vector<int>& ids,
        int start, int end, int depth)
    {
        if (start >= end)
            return nullptr;

        int k = points[0].size();   // cordinate dimension(2D, 3D, etc)
        int axis = depth % k;       // current axis: x/y/z

        // Sort indices [start, end) based on axis
        std::sort(ids.begin() + start, ids.begin() + end,
            [&points, axis](int a, int b)
            {
                return points[a][axis] < points[b][axis];
            });

        int mid = start + (end - start) / 2;

        Node* node = new Node(points[ids[mid]], ids[mid]);

        // Recursive build
        node->left = buildBalanced(points, ids, start, mid, depth + 1);
        node->right = buildBalanced(points, ids, mid + 1, end, depth + 1);

        return node;
    }

    // Public interface to build tree
    void build(std::vector<std::vector<float>>& points)
    {
        std::vector<int> ids(points.size());
        for (int i = 0; i < points.size(); i++)
            ids[i] = i;

        root = buildBalanced(points, ids, 0, points.size(), 0);
    }

};




