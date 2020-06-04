/**
 * PathFinding.h
 * Purpose: Path Finding Algorithm
 * @author Mintae Kim
 */

#pragma once

#include <utility>
#include <vector>

#include "../struct/vector/Vector.h"

#include "../struct/states/Grid.h"

std::vector<std::pair<Vector2D, Vector2D>> pathFinding(
	std::pair<Vector2D, Vector2D> pose,
	std::pair<Vector2D, Vector2D> finalPose,
	Grid *grid, 
	std::unique_lock<std::mutex>& lck, 
	std::condition_variable &cv);