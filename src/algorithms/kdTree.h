// KD-Tree implementation

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root;

	KdTree() : root(nullptr) {}
	void insert(std::vector<float> point, int id);
	std::vector<int> search(std::vector<float> target, float distanceTol);

	private:
		void _insertHelper(Node* &cur, Node* node, uint depth=0);
		void _searchHelper(Node* &cur, std::vector<float> &target, std::vector<int> &ids, float distanceTol, uint depth=0);
};




