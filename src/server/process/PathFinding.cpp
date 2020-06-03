/**
 * PathFinding.cpp
 * Purpose: Path Finding Algorithm
 * @author Mintae Kim
 * @author Youngtae Moon
 */

#include "PathFinding.h"

std::vector<std::pair<Position2D, Direction2D>> pathFinding (
	std::pair<Position2D, Direction2D> pose, 
	std::pair<Position2D, Direction2D> finalPose, 
	Grid* grid, 
	std::unique_lock<std::mutex>& lck, 
	std::condition_variable& cv)
{
	std::vector<std::pair<Position2D, Direction2D>> path;
	
	// TODO: Implement

	return path;
}	