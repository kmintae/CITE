/**
 * BrickLayerList.cpp
 * Purpose: Shared Class 'BrickLayerList': Manages Total Brick Layer & List
 * @author Mintae Kim
 */

#include "BrickLayerList.h"

BrickLayerList::BrickLayerList(std::mutex *mtx, std::condition_variable* cv)
{
	JsonParser jsonParser = JsonParser(); // Loading Json

	srcBrickLayerList = jsonParser.getSrcBrickLayerList();
	dstBrickLayerList = jsonParser.getDstBrickLayerList();

	this->mtx = mtx;
	this->cv = cv;

	// Initialization
	srcBrickLayerIndex = dstBrickLayerIndex = 0;
}
BrickLayerList::~BrickLayerList()
{
	// De-Allocate Brick Layers
	for (int i = 0; i < srcBrickLayerList.size(); i++)
		delete srcBrickLayerList[i];
	for (int i = 0; i < dstBrickLayerList.size(); i++)
		delete dstBrickLayerList[i];
}

std::vector<BrickLayer*>& BrickLayerList::getSrcBrickLayerList()
{
	return srcBrickLayerList;
}
std::vector<BrickLayer*>& BrickLayerList::getDstBrickLayerList()
{
	return dstBrickLayerList;
}

Brick* BrickLayerList::getNextSrcBrick(Robot* robot, int &srcBrickLayerIndex, std::unique_lock<std::mutex> &lck)
{
	// Searching by O(N)
	// Considering LayerList
	Brick* brick = NULL;
	std::pair<Vector2D, Vector2D> pose = robot->getPose();

	while (true)
	{
		// Is Current Layer Jobs Done?
		if (srcBrickLayerList[this->srcBrickLayerIndex]->isDone()) {
			if (this->srcBrickLayerIndex == srcBrickLayerList.size() - 1) return brick; // Jobs Done
			else {
				this->srcBrickLayerIndex++;
				srcBrickLayerIndex = this->srcBrickLayerIndex;
			}
		}
		
		// Select the Nearest Block
		float minDist = 999999999;
		std::vector<Brick*> candidates = srcBrickLayerList[this->srcBrickLayerIndex]->getEnableBrickList();
		for (int i = 0; i < candidates.size(); i++) {
			float dist = Vector2D::calculateDistance(pose.first, candidates[i]->getPos2D());
			if (minDist > dist) {
				minDist = dist;
				brick = candidates[i];
			}
		}
		
		if (brick == NULL) (*cv).wait(lck);
		else break;
	}
	return brick;
}
Brick* BrickLayerList::getNextDstBrick(Robot* robot, int &dstBrickLayerIndex, std::unique_lock<std::mutex>& lck)
{
	// Searching by O(N)
	// Considering LayerList
	Brick* brick = NULL;
	std::pair<Vector2D, Vector2D> pose = robot->getPose();

	while (true)
	{
		// Is Current Layer Jobs Done?
		if (dstBrickLayerList[this->dstBrickLayerIndex]->isDone()) {
			if (this->dstBrickLayerIndex == dstBrickLayerList.size() - 1) return brick; // Jobs Done
			else {
				this->dstBrickLayerIndex++;
				dstBrickLayerIndex = this->dstBrickLayerIndex;
			}
		}

		// Select the Nearest Block
		float minDist = 999999999;
		std::vector<Brick*> candidates = dstBrickLayerList[this->dstBrickLayerIndex]->getEnableBrickList();
		for (unsigned int i = 0; i < candidates.size(); i++) {
			float dist = Vector2D::calculateDistance(pose.first, candidates[i]->getPos2D());
			if (minDist > dist) {
				minDist = dist;
				brick = candidates[i];
			}
		}
		
		if (brick == NULL) (*cv).wait(lck);
		else break;
	}
	return brick;
}

std::pair<Vector2D, Vector2D> BrickLayerList::getFinalPose(Grid* grid, Brick* brick, std::unique_lock<std::mutex>& lck)
{
	std::pair<Vector2D, Vector2D> result;

	int dist = GetPrivateProfileInt("calc", "GRAB_DIST", -1, "../config/server.ini");
	// Get Final Pose w/ Grid & Brick Information
	// Assumption: There are at least one position for releasing specific brick.
	
	Vector2D dirVector = brick->getDir2D();
	Vector2D brickPos = brick->getPos2D();
	Vector2D normVector = Vector2D(dirVector.y, -dirVector.x);

	int gripperDist = GetPrivateProfileInt("physical", "GRIPPER_MM", 2, "../config/server.ini");

	Vector2D grabCandi1 = brickPos - (dirVector * dist);
	grabCandi1 -= (normVector * gripperDist);
	Vector2D grabCandi2 = brickPos + (dirVector * dist);
	grabCandi2 += (normVector * gripperDist);

	// Check index-out-of-range
	bool chk1 = true;
	bool chk2 = true;

	if (grabCandi1.x < 0 || grabCandi1.x >= grid->limit_x || grabCandi1.y < 0 || grabCandi1.y >= grid->limit_y) chk1 = false;
	if (grabCandi2.x < 0 || grabCandi2.x >= grid->limit_x || grabCandi2.y < 0 || grabCandi2.y >= grid->limit_y) chk2 = false;

	bool** grid_bool = grid->getGrid();
	while (true)
	{	
		// If no finalpose exists, then *cv.wait(lck);
		// Assumption: + High Priority

		if (chk2 == true)
			if (grid_bool[(int)(grabCandi2.y / grid->grid_len)][(int)(grabCandi2.x / grid->grid_len)] == false)
				return std::make_pair(grabCandi2, dirVector * -1);

		if (chk1 == true)
			if (grid_bool[(int)(grabCandi1.y / grid->grid_len)][(int)(grabCandi1.x / grid->grid_len)] == false)
				return std::make_pair(grabCandi1, dirVector);


		(*cv).wait(lck);
	}
	return result;
}

void BrickLayerList::markAsSelectedBrick(int srcBrickLayerIndex, Brick* brick)
{
	srcBrickLayerList[srcBrickLayerIndex]->markAsSelected(brick);
}
void BrickLayerList::markAsGrabbedBrick(Brick* brick)
{
	brick->setAsGrabbed();
}
void BrickLayerList::markAsLiftedBrick(Brick* srcBrick, int dstBrickLayerIndex, Brick* dstBrick)
{
	srcBrick->setAsLifted();
	dstBrickLayerList[dstBrickLayerIndex]->markAsSelected(dstBrick);
}
void BrickLayerList::markAsReleasedBrick(Brick* brick)
{
	brick->setAsReleased();
}
void BrickLayerList::markAsDone(int srcBrickLayerIndex, Brick *srcBrick, int dstBrickLayerIndex, Brick *dstBrick)
{
	srcBrickLayerList[srcBrickLayerIndex]->markAsDone(srcBrick);
	dstBrickLayerList[dstBrickLayerIndex]->markAsDone(dstBrick);
}

bool BrickLayerList::isDone() // 하자 있는 곳
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(*mtx);

	int bricksTotal = 0;
	for (int i = 0; i < dstBrickLayerList.size(); i++) {
		bricksTotal += dstBrickLayerList[i]->getBrickList().size();
	}

	int bricksDone = 0;
	for (int i = 0; i < dstBrickLayerIndex; i++) {
		bricksDone += dstBrickLayerList[i]->getStackedList().size();
	}
	for (int i = 0; i < dstBrickLayerList[dstBrickLayerIndex]->getBrickList().size(); i++) {
		if (dstBrickLayerList[dstBrickLayerIndex]->getBrickList()[i]->getPhase() == BrickPhase::DONE)
			bricksDone++;
	}

	if (bricksDone == bricksTotal) return true;
	else return false;
}

float BrickLayerList::getProgressRate()
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(*mtx);

	int bricksTotal = 0;
	for (int i = 0; i < dstBrickLayerList.size(); i++) {
		bricksTotal += dstBrickLayerList[i]->getBrickList().size();
	}

	int bricksDone = 0;
	for (int i = 0; i < dstBrickLayerIndex; i++) {
		bricksDone += dstBrickLayerList[i]->getStackedList().size();
	}
	for (int i = 0; i < dstBrickLayerList[dstBrickLayerIndex]->getBrickList().size(); i++) {
		if (dstBrickLayerList[dstBrickLayerIndex]->getBrickList()[i]->getPhase() == BrickPhase::DONE)
			bricksDone++;
	}

	return (float)bricksDone * 100.0 / (float)bricksTotal;
}