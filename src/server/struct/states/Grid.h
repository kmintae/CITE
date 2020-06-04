/**
 * Grid.h
 * Purpose: Shared Class 'Grid': Expressing Current State
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <mutex>
#include <thread>
#include <Windows.h>
#include <vector>

#include "../basic_blocks/Brick.h"
#include "../basic_blocks/BrickLayer.h"
#include "../basic_blocks/Robot.h"

#include "../connection/Client.h"

#include "../parser/OptitrackCommunicator.h"

#include "../vector/Vector.h"

// Forward Declaration
class BrickLayerList;

class Grid
{
private:
	int grid_len;
	int limit_x, limit_y;
	bool** grid_bool;
	
	int robotBorder, brickBorder;
	int errorLimit;

	int maxRobotNum;
	std::mutex* mtx;
	std::condition_variable* cv;

public:
	int x, y;
	Grid(std::mutex* mtx, std::condition_variable* cv);
	~Grid();

	bool** getGrid();

	void repaint(BrickLayerList *brickLayerList, OptitrackCommunicator* optitrackCommunicator, Client** clients);
};