/**
 * Robot.h
 * Purpose: Class encapsulates 'Robot' Information
 * @author Mintae Kim
 */

#pragma once

#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <vector>

#include "Phase.h"
#include "Brick.h"
#include "BrickLayer.h"
#include "Instruction.h"

#include "../vector/Vector.h"

class Robot
{
private:
	int robotNum;
	
	std::pair<Vector2D, Vector2D> pose, finalPose;
	std::vector<std::pair<Vector2D, Vector2D>> path;
	Brick *src, *dst;

	// Lock: Need Information?
	std::mutex mtx;

public:
	RobotPhase phase;

	Robot(int robotNum, std::pair<Vector2D, Vector2D>& initPose);

	int getRobotNum();

	void setPose(const std::pair<Vector2D, Vector2D> &pose);
	std::pair<Vector2D, Vector2D>& getPose();
	void setPath(const std::vector<std::pair<Vector2D, Vector2D>>& path);
	std::vector<std::pair<Vector2D, Vector2D>>& getPath();
	std::pair<Vector2D, Vector2D>& getKeypoint();
	
	std::pair<Vector2D, Vector2D>& getFinalPose();

	Brick* getSourceBrick();
	Brick* getDestinationBrick();

	void markAsMove(Brick* srcBrick, const std::pair<Vector2D, Vector2D>& finalPose);
	void markAsGrab();
	void markAsLift(Brick* dstBrick, const std::pair<Vector2D, Vector2D>& finalPose);
	void markAsRelease();
	void markAsStop();

	std::string toString();
};