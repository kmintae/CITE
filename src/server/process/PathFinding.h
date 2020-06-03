/**
 * PathFinding.h
 * Purpose: Path Finding Algorithm
 * @author Mintae Kim
 */

#pragma once

#include <utility>
#include <vector>

#include "../struct/vector/Vector.h"
#include "../struct/vector/Position.h"
#include "../struct/vector/Direction.h"

#include "../struct/states/Grid.h"

std::vector<std::pair<Position2D, Direction2D>> pathFinding(
	std::pair<Position2D, Direction2D> pose, 
	std::pair<Position2D, Direction2D> finalPose, 
	Grid *grid, 
	std::unique_lock<std::mutex>& lck, 
	std::condition_variable &cv);