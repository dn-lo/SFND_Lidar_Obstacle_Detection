// KD-Tree implementation

#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>
#include <cmath>
#include <iostream>
typedef unsigned int uint;

// Class to represent node of KD-Ttree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(nullptr), right(nullptr) {}
};

// KdTree class
struct KdTree
{
	Node* root;

	KdTree() : root(nullptr) {}
	~KdTree();

	void insert(std::vector<float> point, int id);
	std::vector<int> search(std::vector<float> target, float distanceTol);

	void deleteSubtree(Node* subtreeRootPtr);

	private:
		void _insertHelper(Node* &cur, Node* node, uint depth=0);
		void _searchHelper(Node* &cur, std::vector<float> &target, std::vector<int> &ids, float distanceTol, uint depth=0);
};

#endif



