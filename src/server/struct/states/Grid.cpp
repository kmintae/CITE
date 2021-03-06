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

	limit_x = GetPrivateProfileInt("limit", "X_MM", -1, "../config/server.ini");
	limit_y = GetPrivateProfileInt("limit", "Y_MM", -1, "../config/server.ini");
	grid_len = GetPrivateProfileInt("grid", "GRID_LEN_MM", 0, "../config/server.ini");

	robotBorder = 2 * GetPrivateProfileInt("physical", "ROBOT_BODY_SIZE_MM", -1, "../config/server.ini");
	brickBorder = GetPrivateProfileInt("grid", "BRICK_GRID_MM", -1, "../config/server.ini");
	errorLimit = GetPrivateProfileInt("error", "POS_LIMIT_MM", -1, "../config/server.ini");

	maxRobotNum = GetPrivateProfileInt("connection", "MAX_ROBOT_CONNECTED", -1, "../config/server.ini");

	x = (limit_x / grid_len + 2);
	y = (limit_y / grid_len + 2);

	// Dynamic Allocation
	grid_bool = new bool* [y];
	
	for (int i = 0; i < y; i++) grid_bool[i] = new bool[x];

	// Initialization
	for (int i = 0; i < y; i++) {
		for (int j = 0; j < x; j++) {
			grid_bool[i][j] = false;
		}
	}
}

Grid::~Grid()
{
	for (int i = 0; i < y; i++) delete [] grid_bool[i];
	delete[] grid_bool;
}

bool** Grid::getGrid()
{
	return grid_bool;
}

void Grid::repaint(BrickLayerList* brickLayerList, OptitrackCommunicator* optitrackCommunicator, Client** clients)
{
	// DRAW GRID! (BrickList, Optitrack Infos, Client List (Path))
	for (int i = 0; i < y; i++) {
		for (int j = 0; j < x; j++) grid_bool[i][j] = false;
	}

	// 1. BrickLayerList
	std::vector<BrickLayer*> srcBrickLayerList = brickLayerList->getSrcBrickLayerList();
	std::vector<BrickLayer*> dstBrickLayerList = brickLayerList->getDstBrickLayerList();
	
	for (int i = 0; i < srcBrickLayerList.size(); i++) {
		std::vector<Brick*>& srcBrickList = srcBrickLayerList[i]->getBrickList();
		for (int j = 0; j < srcBrickList.size(); j++) {
			if (srcBrickList[j]->getPhase() == BrickPhase::DONE ||
				srcBrickList[j]->getPhase() == BrickPhase::RELEASED) continue;
			
			Vector2D brickPos = srcBrickList[j]->getPos2D();
			for (int k = (((brickPos.y - brickBorder - 1) / grid_len >= 0) ? (brickPos.y - brickBorder - 1) / grid_len : 0);
				k <= (((brickPos.y + brickBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (brickPos.y + brickBorder + 1) / grid_len : (limit_y - 1) / grid_len);
				k++) {
				for (int l = (((brickPos.x - brickBorder - 1) / grid_len >= 0) ? (brickPos.x - brickBorder - 1) / grid_len : 0);
					l <= (((brickPos.x + brickBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (brickPos.x + brickBorder + 1) / grid_len : (limit_x - 1) / grid_len);
					l++) {
					// TODO: (l,, k)) �ƴ�
					Vector2D newVect = Vector2D(l * grid_len + grid_len / 2, k * grid_len + grid_len / 2);
					if (Vector2D::calculateDistance(newVect, brickPos) < brickBorder) grid_bool[k][l] = true;
				}
			}
		}
	}

	for (int i = 0; i < dstBrickLayerList.size(); i++) {
		std::vector<Brick*>& dstBrickList = dstBrickLayerList[i]->getBrickList();
		for (int j = 0; j < dstBrickList.size(); j++) {
			if (dstBrickList[j]->getPhase() != BrickPhase::DONE) continue;
			
			Vector2D brickPos = dstBrickList[j]->getPos2D(); // Should not be reference typed
			for (int k = (((brickPos.y - brickBorder - 1) / grid_len >= 0) ? (brickPos.y - brickBorder - 1) / grid_len : 0);
				k <= (((brickPos.y + brickBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (brickPos.y + brickBorder + 1) / grid_len : (limit_y - 1) / grid_len);
				k++) {
				for (int l = (((brickPos.x - brickBorder - 1) / grid_len >= 0) ? (brickPos.x - brickBorder - 1) / grid_len : 0);
					l <= (((brickPos.x + brickBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (brickPos.x + brickBorder + 1) / grid_len : (limit_x - 1) / grid_len);
					l++) {
					Vector2D newVect = Vector2D(l * grid_len + grid_len / 2, k * grid_len + grid_len / 2);
					if (Vector2D::calculateDistance(newVect, brickPos) < brickBorder) grid_bool[k][l] = true;
				}
			}
		}
	}

	char buf[512] = { 0. };
	GetPrivateProfileString("error", "CENTER_DIST", "-1", buf, 512, "../config/server.ini");
	float centerError = atof(buf);

	// 2. OptitrackCommunicator (Position Data)
	std::pair<Vector2D, Vector2D>* poseArr = optitrackCommunicator->getPoseArray();
	for (int i = 0; i < maxRobotNum; i++) {
		if (clients[i]->connectedHistory == false) continue;
		Vector2D robotPos = poseArr[i].first;
		Vector2D dir = poseArr[i].second;

		// Callibration: Center of Wheels -> Optitrack
		float x = robotPos.x-dir.x*centerError;
		float y = robotPos.y - dir.y * centerError;
		for (int k = (((y - robotBorder - 1) / grid_len >= 0) ? (y - robotBorder - 1) / grid_len : 0);
			k <= (((y + robotBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (y + robotBorder + 1) / grid_len : (limit_y - 1) / grid_len);
			k++) {
			for (int l = (((x - robotBorder - 1) / grid_len >= 0) ? (x - robotBorder - 1) / grid_len : 0);
				l <= (((x + robotBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (x + robotBorder + 1) / grid_len : (limit_x - 1) / grid_len);
				l++) {
				Vector2D newVect = Vector2D(l * grid_len + grid_len / 2, k * grid_len + grid_len / 2);
				if (Vector2D::calculateDistance(newVect, robotPos) < robotBorder) grid_bool[k][l] = true;
			}
		}
	}

	delete[] poseArr;
	
	// 3. Path Data (client, MOVING/LIFTING phase)
	for (int i = 0; i < maxRobotNum; i++) {
		if (clients[i]->phase == ClientPhase::ACCEPTED) {
			Robot* robot = clients[i]->getRobot();
			if (robot->phase == RobotPhase::MOVING || robot->phase == RobotPhase::LIFTING) {
				Vector2D pose = robot->getPose().first;
				
				// Callibration: Center of Wheels -> Optitrack
				Vector2D dir = poseArr[i].second;
				float x = pose.x - dir.x * centerError;
				float y = pose.y - dir.y * centerError;
				Vector2D pose_cal = Vector2D(x, y);

				std::pair<Vector2D, Vector2D> keypoint = robot->getKeypoint();
				
				if (Vector2D::isNearest(pose_cal, keypoint.first)) return;

				// In CITD IV, We'll assume that Robot moves with 4 directions
				// This can be modified by using: Rotation Transformation
				if (abs(x - keypoint.first.x) < errorLimit) { // Parallel to X
					int y_min = ((y < keypoint.first.y) ? y : keypoint.first.y);
					int y_max = ((y < keypoint.first.y) ? keypoint.first.y : y);

					for (int k = y_min / grid_len; k <= y_max / grid_len; k++) {
						for (int l = (((x - robotBorder - 1) / grid_len >= 0) ? (x - robotBorder - 1) / grid_len : 0);
							l <= (((x + robotBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (x + robotBorder + 1) / grid_len : (limit_x - 1) / grid_len);
							l++) {
							grid_bool[k][l] = true;
						}
					}
				}
				else if (abs(y - keypoint.first.y) < errorLimit) { // Parallel to Y
					int x_min = ((x < keypoint.first.x) ? x : keypoint.first.x);
					int x_max = ((x < keypoint.first.x) ? keypoint.first.x : x);

					for (int k = (((y - robotBorder - 1) / grid_len >= 0) ? (y - robotBorder - 1) / grid_len : 0);
						k <= (((y + robotBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (y + robotBorder + 1) / grid_len : (limit_y - 1) / grid_len);
						k++) {
						for (int l = x_min / grid_len; l <= x_max / grid_len; l++) {
							grid_bool[k][l] = true;
						}
					}
				}

				// Block around keypoint
				for (int k = (((keypoint.first.y - robotBorder - 1) / grid_len >= 0) ? (keypoint.first.y - robotBorder - 1) / grid_len : 0);
					k <= (((keypoint.first.y + robotBorder + 1) / grid_len <= (limit_y - 1) / grid_len) ? (keypoint.first.y + robotBorder + 1) / grid_len : (limit_y - 1) / grid_len);
					k++) {
					for (int l = (((keypoint.first.x - robotBorder - 1) / grid_len >= 0) ? (keypoint.first.x - robotBorder - 1) / grid_len : 0);
						l <= (((keypoint.first.x + robotBorder + 1) / grid_len <= (limit_x - 1) / grid_len) ? (keypoint.first.x + robotBorder + 1) / grid_len : (limit_x - 1) / grid_len);
						l++) {
						Vector2D newVect = Vector2D(l * grid_len + grid_len / 2, k * grid_len + grid_len / 2);
						if (Vector2D::calculateDistance(newVect, keypoint.first) < robotBorder) grid_bool[k][l] = true;
					}
				}
			}
		}
	}

	cv->notify_all();
}