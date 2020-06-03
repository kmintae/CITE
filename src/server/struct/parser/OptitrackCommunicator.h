/**
 * OptitrackCommunicator.h
 * Purpose: Communicator w/ Optitrack, Holding Position & Direction Data
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <thread>
#include <mutex>
#include <utility>
#include <string>
#include <cmath>
#include <sstream>
#include <WinSock2.h>
#include <Windows.h>

#include "../../process/OptitrackCommunicate.h"

#include "../vector/Position.h"
#include "../vector/Direction.h"

#define CAL_ROT_DEG 22.5

class OptitrackCommunicator
{
private:
	int maxRobotLimit;

	std::thread communicator;

	std::pair<Position2D, Direction2D>* poseArr; // Shared: Heap

	static std::mutex mtx; // Shared
	static bool isDestructed; // Shared

	void updateArray(std::string rawData);

	static SOCKET udpSocket;

public:
	OptitrackCommunicator();
	~OptitrackCommunicator();

	void communicate(); // Parsing Data

	std::pair<Position2D, Direction2D>* getPoseArray();

	std::pair<Position2D, Direction2D> getPose(int robotNum);

	std::pair<int, std::pair<Position2D, Direction2D>> getConnectWaitingRobot (std::pair<Position2D, Direction2D>* prevPoseArr);
};