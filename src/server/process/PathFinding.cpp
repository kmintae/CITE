/**
 * PathFinding.cpp
 * Purpose: Path Finding Algorithm
 * @author Mintae Kim
 * @author Youngtae Moon
 */

#include "PathFinding.h"

std::vector<std::pair<Vector2D, Vector2D>> pathFinding(
	std::pair<Vector2D, Vector2D> pose,
	std::pair<Vector2D, Vector2D> finalPose,
	Grid* grid,
	std::unique_lock<std::mutex>& lck,
	std::condition_variable& cv)
{
	std::vector<std::pair<Vector2D, Vector2D>> path_reverse, path;
	/* TODO: FIX
	int pos_y = pose.first.y / grid->grid_len;
	int pos_x = pose.first.x / grid->grid_len;
	int fin_y = finalPose.first.y / grid->grid_len;
	int fin_x = finalPose.first.x / grid->grid_len;
	int min_dist = INF;
	int final_k=0;
	bool** grid_bool = grid->getGrid();

	while (true) {
		arr = new Node **[grid->y];
		for (int i = 0; i < grid->y; i++) {
			arr[i] = new Node * [grid->x];
			for (int j = 0; j < grid->x; j++) {
				arr[i][j] = new Node[4];
				for (int k = 0; k < 4; k++) {
					arr[i][j][k].y = i;
					arr[i][j][k].x = j;
					arr[i][j][k].rotation = k;
				}
			}
		}

		for (int i = 0; i < grid->y; i++) {
			for (int j = 0; j < grid->x; j++) {
				for (int k = 0; k < 4; k++) {
					int x = 0;
					int y = 0;
					switch (k) {
					case 0:
						y = i + 1;
						x = j;
						break;
					case 1:
						y = i;
						x = j + 1;
						break;
					case 2:
						y = i - 1;
						x = j;
						break;
					case 3:
						y = i;
						x = j - 1;
						break;
					default:
						break;
					}
					if (x < 0) {
						x = 0;
					}
					if (y < 0) {
						y = 0;
					}
					if (y == grid->y) {
						y = grid->y - 1;
					}
					if (x == grid->x) {
						x = grid->x - 1;
					}
					arr[i][j][k].edge.push_back(arr[y][x][k]);
					arr[i][j][k].edge.push_back(arr[i][j][(k + 1) % 4]);
					arr[i][j][k].edge.push_back(arr[i][j][(k + 2) % 4]);
					arr[i][j][k].edge.push_back(arr[i][j][(k + 3) % 4]);
					if (pos_x == j && pos_y == i && k == 1) arr[i][j][k].dist = 0;
					else arr[i][j][k].dist = INF;

				}
			}
		}	
		q.push(&arr[pos_y][pos_x][1]);

		while (!q.empty()) {
			Node* n =q.top();
			q.pop();
			for (auto e : n->edge) {
				int cost = n->dist + 1;
				if (e.dist > cost && !grid_bool[e.y][e.x]) {
					e.dist = cost;
					q.push(&e);
					e.before = n;
				}
			}
		}
		for (int k = 0;k < 4;k++) {
			if (arr[fin_y][fin_x][k].dist < min_dist) {
				final_k = k;
				min_dist = arr[fin_y][fin_x][k].dist;
			}
		}

		if (min_dist == INF) {
			cv.wait(lck); // 못찾을경우.
		}
		else {
			if (!Vector2D::isSimilar(Vector2D::radianToVector((((final_k + 3) % 4) * 90) * 3.141592 / 180.0), finalPose.second)) {
				path_reverse.push_back(finalPose);
			}

			Node* next = &arr[fin_y][fin_x][final_k];
			while (true) {
				if (next->rotation != next->before->rotation) path_reverse.push_back(std::make_pair(Vector2D(fin_y, fin_x), Vector2D::radianToVector((((final_k + 3) % 4) * 90) * 3.141592 / 180.0)));
				next = next->before;
			}

			while (!path_reverse.empty()) {
				path.push_back(path_reverse.back());
				path_reverse.pop_back();
			}
			break;
		}
	}
	*/
	return path;
}	