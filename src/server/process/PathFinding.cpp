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
	std::vector<std::pair<Vector2D, Vector2D>> path;
	
	// TODO: Implement

	return path;
}	