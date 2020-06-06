/**
 * PathFinding.h
 * Purpose: Path Finding Algorithm
 * @author Mintae Kim
 */

#pragma once

#include <utility>
#include <vector>
#include <queue>
#include "../struct/vector/Vector.h"

#include "../struct/states/Grid.h"

#define INF 999999
/*
typedef struct temp
{
	int x, y;
	int rotation;
	int dist;
	std::vector<Node> edge;
	struct temp * before;
} Node;
struct cmp {
	bool operator()(Node* n1, Node* n2) {
		return n1->dist > n2->dist;
	}
};
Node*** arr;
int* d;

std::priority_queue<Node *, std::vector<Node *>, cmp> q;
*/
std::vector<std::pair<Vector2D, Vector2D>> pathFinding(
	std::pair<Vector2D, Vector2D> pose,
	std::pair<Vector2D, Vector2D> finalPose,
	Grid *grid, 
	std::unique_lock<std::mutex>& lck, 
	std::condition_variable &cv);