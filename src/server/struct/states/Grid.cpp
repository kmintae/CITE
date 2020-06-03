/**
 * Grid.cpp
 * Purpose: Shared Class 'Grid': Expressing Current State
 * @author Mintae Kim
 */

#include "Grid.h"

// Forward Declaration
#include "BrickLayerList.h"

Grid::Grid(std::mutex* mtx, std::condition_variable* cv)
{
	this->mtx = mtx;
	this->cv = cv;

	limit_x = GetPrivateProfileInt("limit", "X_MM", -1, "../../../config/server.ini");
	limit_y = GetPrivateProfileInt("limit", "Y_MM", -1, "../../../config/server.ini");
	grid_len = GetPrivateProfileInt("grid", "GRID_LEN_MM", 0, "../../../config/server.ini");

	robotBorder = GetPrivateProfileInt("physical", "ROBOT_BODY_SIZE_MM", -1, "../../../config/server.ini");
	brickBorder = GetPrivateProfileInt("grid", "BRICK_GRID_MM", -1, "../../../config/server.ini");
	errorLimit = GetPrivateProfileInt("error", "POS_LIMIT_MM", -1, "../../../config/server.ini");

	maxRobotNum = GetPrivateProfileInt("connection", "MAX_ROBOT_CONNECTED", -1, "../../../config/server.ini");

	x = (limit_x / grid_len + 2);
	y = (limit_y / grid_len + 2);

	// Dynamic Allocation
	grid = new bool* [y];
	
	for (int i = 0; i < y; i++) grid[i] = new bool[x];
}

Grid::~Grid()
{
	for (int i = 0; i < y; i++) delete [] grid[i];
	delete[] grid;
}

bool** Grid::getGrid()
{
	return grid;
}

void Grid::repaint(BrickLayerList* brickLayerList, OptitrackCommunicator* optitrackCommunicator, Client** clients)
{
	// TODO: DRAW GRID! (BrickList, Optitrack Infos, Client List (Path))
	// 1. BrickLayerList
	std::vector<BrickLayer*> srcBrickLayerList = brickLayerList->getSrcBrickLayerList();
	std::vector<BrickLayer*> dstBrickLayerList = brickLayerList->getDstBrickLayerList();
	
	for (int i = 0; i < srcBrickLayerList.size(); i++) {
		std::vector<Brick*>& srcBrickList = srcBrickLayerList[i]->getBrickList();
		for (int j = 0; j < srcBrickList.size(); j++) {
			Position2D& brickPos = srcBrickList[i]->getPos2D();
			for (int k = (((brickPos.y - brickBorder - 1) / grid_len >= 0) ? (brickPos.y - brickBorder - 1) / grid_len : 0);
				k <= (((brickPos.y + brickBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (brickPos.y + brickBorder + 1) / grid_len : (limit_y - 1) / grid_len);
				k++) {
				for (int l = (((brickPos.x - brickBorder - 1) / grid_len >= 0) ? (brickPos.x - brickBorder - 1) / grid_len : 0);
					l <= (((brickPos.x + brickBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (brickPos.x + brickBorder + 1) / grid_len : (limit_x - 1) / grid_len);
					l++) {
					if (Position2D::calculateDistance(Position2D(l, k), brickPos) < brickBorder) grid[k][l] = true;
				}
			}
		}
	}

	for (int i = 0; i < dstBrickLayerList.size(); i++) {
		std::vector<Brick*>& dstBrickList = dstBrickLayerList[i]->getBrickList();
		for (int j = 0; j < dstBrickList.size(); j++) {
			Position2D& brickPos = dstBrickList[i]->getPos2D();
			for (int k = (((brickPos.y - brickBorder - 1) / grid_len >= 0) ? (brickPos.y - brickBorder - 1) / grid_len : 0);
				k <= (((brickPos.y + brickBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (brickPos.y + brickBorder + 1) / grid_len : (limit_y - 1) / grid_len);
				k++) {
				for (int l = (((brickPos.x - brickBorder - 1) / grid_len >= 0) ? (brickPos.x - brickBorder - 1) / grid_len : 0);
					l <= (((brickPos.x + brickBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (brickPos.x + brickBorder + 1) / grid_len : (limit_x - 1) / grid_len);
					l++) {
					if (Position2D::calculateDistance(Position2D(l, k), brickPos) < brickBorder) grid[k][l] = true;
				}
			}
		}
	}

	// 2. OptitrackCommunicator (Position Data)
	std::pair<Position2D, Direction2D>* poseArr = optitrackCommunicator->getPoseArray();
	for (int i = 0; i < maxRobotNum; i++) {
		Position2D& robotPos = poseArr[i].first;
		for (int k = (((robotPos.y - robotBorder - 1) / grid_len >= 0) ? (robotPos.y - robotBorder - 1) / grid_len : 0);
			k <= (((robotPos.y + robotBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (robotPos.y + robotBorder + 1) / grid_len : (limit_y - 1) / grid_len);
			k++) {
			for (int l = (((robotPos.x - robotBorder - 1) / grid_len >= 0) ? (robotPos.x - robotBorder - 1) / grid_len : 0);
				l <= (((robotPos.x + robotBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (robotPos.x + robotBorder + 1) / grid_len : (limit_x - 1) / grid_len);
				l++) {
				if (Position2D::calculateDistance(Position2D(l, k), robotPos) < robotBorder) grid[k][l] = true;
			}
		}
	}

	delete[] poseArr;
	
	// 3. Path Data (client, MOVING/LIFTING phase)
	for (int i = 0; i < maxRobotNum; i++) {
		if (clients[i]->phase == ClientPhase::ACCEPTED) {
			Robot* robot = clients[i]->getRobot();
			if (robot->phase == RobotPhase::MOVING || robot->phase == RobotPhase::LIFTING) {
				Position2D& pose = robot->getPose().first;
				Position2D& keypoint = robot->getKeypoint().first;
				
				if (pose == keypoint) return;

				// In CITD IV, We'll assume that Robot moves with 4 directions
				// This can be modified by using: Rotation Transformation
				if (abs(pose.x - keypoint.x) < errorLimit) { // Parallel to X
					int y_min = ((pose.y < keypoint.y) ? pose.y : keypoint.y);
					int y_max = ((pose.y < keypoint.y) ? keypoint.y : pose.y);

					for (int k = y_min / grid_len; k <= y_max / grid_len; k++) {
						for (int l = (((pose.x - robotBorder - 1) / grid_len >= 0) ? (pose.x - robotBorder - 1) / grid_len : 0);
							l <= (((pose.x + robotBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (pose.x + robotBorder + 1) / grid_len : (limit_x - 1) / grid_len);
							l++) {
							grid[k][l] = true;
						}
					}
				}
				else if (abs(pose.y - keypoint.y) < errorLimit) { // Parallel to Y
					int x_min = ((pose.x < keypoint.x) ? pose.x : keypoint.x);
					int x_max = ((pose.x < keypoint.x) ? keypoint.x : pose.x);

					for (int k = (((pose.y - robotBorder - 1) / grid_len >= 0) ? (pose.y - robotBorder - 1) / grid_len : 0);
						k <= (((pose.y + robotBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (pose.y + robotBorder + 1) / grid_len : (limit_y - 1) / grid_len);
						k++) {
						for (int l = x_min / grid_len; l <= x_max / grid_len; l++) {
							grid[k][l] = true;
						}
					}
				}

				// Block around keypoint
				for (int k = (((keypoint.y - robotBorder - 1) / grid_len >= 0) ? (keypoint.y - robotBorder - 1) / grid_len : 0);
					k <= (((keypoint.y + robotBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (keypoint.y + robotBorder + 1) / grid_len : (limit_y - 1) / grid_len);
					k++) {
					for (int l = (((keypoint.x - robotBorder - 1) / grid_len >= 0) ? (keypoint.x - robotBorder - 1) / grid_len : 0);
						l <= (((keypoint.x + robotBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (keypoint.x + robotBorder + 1) / grid_len : (limit_x - 1) / grid_len);
						l++) {
						if (Position2D::calculateDistance(Position2D(l, k), keypoint) < robotBorder) grid[k][l] = true;
					}
				}
			}
		}
	}

	cv->notify_all();
}