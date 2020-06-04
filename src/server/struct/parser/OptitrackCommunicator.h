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

#include "../vector/Vector.h"

#define CAL_ROT_DEG 22.5

class OptitrackCommunicator
{
private:
	static int maxRobotLimit;

	std::thread communicator;

	static std::pair<Vector2D, Vector2D>* poseArr; // Shared: Heap

	static std::mutex mtx; // Shared
	static bool isDestructed; // Shared

	void updateArray(std::string rawData);

	static SOCKET udpSocket;

	WSAData wsaData;

public:
	OptitrackCommunicator();
	~OptitrackCommunicator();

	void communicate(); // Parsing Data

	std::pair<Vector2D, Vector2D>* getPoseArray();

	std::pair<Vector2D, Vector2D> getPose(int robotNum);

	std::pair<int, std::pair<Vector2D, Vector2D>> getConnectWaitingRobot (std::pair<Vector2D, Vector2D>* prevPoseArr);
};