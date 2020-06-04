/**
 * BrickLayerList.h
 * Purpose: Shared Class 'BrickLayerList': Manages Total Brick Layer & List
 * @author Mintae Kim
 */

#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include "../basic_blocks/Phase.h"
#include "../basic_blocks/Brick.h"
#include "../basic_blocks/Robot.h"

#include "../parser/JsonParser.h"

#include "Grid.h"

#include "../vector/Vector.h"

class BrickLayerList
{
private:
	std::vector<BrickLayer*> srcBrickLayerList, dstBrickLayerList;
	int srcBrickLayerIndex, dstBrickLayerIndex;

	std::mutex* mtx;
	std::condition_variable* cv;

public:
	BrickLayerList(std::mutex* mtx, std::condition_variable *cv);
	~BrickLayerList();

	std::vector<BrickLayer*>& getSrcBrickLayerList();
	std::vector<BrickLayer*>& getDstBrickLayerList();
	Brick* getNextSrcBrick(Robot* robot, int& srcBrickLayerIndex, std::unique_lock<std::mutex>& lck);
	Brick* getNextDstBrick(Robot* robot, int& dstBrickLayerIndex, std::unique_lock<std::mutex>& lck);

	std::pair<Vector2D, Vector2D> getFinalPose(Grid* grid, Brick* brick, std::unique_lock<std::mutex>& lck);

	void markAsSelectedBrick(int srcBrickLayerIndex, Brick* brick);
	void markAsGrabbedBrick(Brick* brick);
	void markAsLiftedBrick(int dstBrickLayerIndex, Brick* brick);
	void markAsReleasedBrick(Brick* brick);
	void markAsDone(int srcBrickLayerIndex, Brick* srcBrick, int dstBrickLayerIndex, Brick* dstBrick);

	bool isDone();
	float getProgressRate();
};