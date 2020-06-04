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
	// TODO: Get Final Pose w/ Grid & Brick Information
	while (true)
	{
		break;
		// TODO: If no finalpose exists, then *cv.wait(lck);
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
void BrickLayerList::markAsLiftedBrick(int dstBrickLayerIndex, Brick* brick)
{
	dstBrickLayerList[dstBrickLayerIndex]->markAsSelected(brick);
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

bool BrickLayerList::isDone()
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