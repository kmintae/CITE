/**
 * OptitrackCommunicator.cpp
 * Purpose: Communicator w/ Optitrack, Holding Position & Direction Data
 * @author Mintae Kim
 * @author Rafat Cieslak (https://rafalcieslak.wordpress.com/2014/05/16/c11-stdthreads-managed-by-a-designated-class/)
 */

#include "OptitrackCommunicator.h"

std::mutex OptitrackCommunicator::mtx; // Shared
bool OptitrackCommunicator::isDestructed = false; // Shared
SOCKET OptitrackCommunicator::udpSocket;
int OptitrackCommunicator::maxRobotLimit = GetPrivateProfileInt("connection", "MAX_ROBOT_CONNECTED", 2, "../config/server.ini");

std::pair<Vector2D, Vector2D>* OptitrackCommunicator::poseArr = new std::pair<Vector2D, Vector2D>[OptitrackCommunicator::maxRobotLimit]; // Shared: Heap

OptitrackCommunicator::OptitrackCommunicator()
{
	// Socket Initiation
	UDPSocket(wsaData, OptitrackCommunicator::udpSocket);
	
	// Multithread function communicate()
	communicator = std::thread(&OptitrackCommunicator::communicate, this);
}
OptitrackCommunicator::~OptitrackCommunicator()
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	// Kill Thread
	OptitrackCommunicator::isDestructed = true;
	communicator.join();

	delete[] OptitrackCommunicator::poseArr;

	WSACleanup();
}

void OptitrackCommunicator::communicate()
{
	while (true)
	{
		if (OptitrackCommunicator::isDestructed) 
		{
			closesocket(OptitrackCommunicator::udpSocket);
			break;
		}

		// Fetching Optitrack Raw Data
		std::string rawData;
		if (fetchOptitrackData(OptitrackCommunicator::udpSocket, rawData) == SOCKET_ERROR) {
			closesocket(OptitrackCommunicator::udpSocket);
			break;
		}
		
		// Update Array
		updateArray(rawData);
	}
}
void OptitrackCommunicator::updateArray(std::string rawData)
{
	// Parse rawData & Update Array
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);
	
	// TODO: Initialize

	std::stringstream ss1(rawData);
	std::string line;
	if (rawData != "")
	{
		while (std::getline(ss1, line, '\n'))
		{
			std::stringstream ss2(line);
			std::string ID_s, X_s, Y_s, theta_s;
			int ID;
			float X, Y, theta;

			// ID
			std::getline(ss2, ID_s, ' ');
			ID = stoi(ID_s);

			// X
			std::getline(ss2, X_s, ' ');
			X = stof(X_s);

			// Y
			std::getline(ss2, Y_s, ' ');
			Y = stof(Y_s);

			// Theta
			std::getline(ss2, theta_s, ' ');
			theta = stof(theta_s);
			
			char buf[512] = { 0. };
			GetPrivateProfileString("error", "CENTER_DIST", "-1", buf, 512, "../config/server.ini");
			float centerError = atof(buf);

			Vector2D dir = Vector2D::radianToVector(theta);
			OptitrackCommunicator::poseArr[ID].first.x = X+centerError*dir.x;
			OptitrackCommunicator::poseArr[ID].first.y = Y+centerError*dir.y;
			OptitrackCommunicator::poseArr[ID].second = dir;
		}
	}
}

// Don't Forget to delete after using get Pos/Dir Array()
std::pair<Vector2D, Vector2D>* OptitrackCommunicator::getPoseArray()
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	std::pair<Vector2D, Vector2D>* newPoseArr = new std::pair<Vector2D, Vector2D>[OptitrackCommunicator::maxRobotLimit];
	for (int i = 0; i < OptitrackCommunicator::maxRobotLimit; i++) {
		newPoseArr[i] = OptitrackCommunicator::poseArr[i];
	}

	return newPoseArr;
}

std::pair<Vector2D, Vector2D> OptitrackCommunicator::getPose(int robotNum)
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	return OptitrackCommunicator::poseArr[robotNum];
}

std::pair<int, std::pair<Vector2D, Vector2D>> OptitrackCommunicator::getConnectWaitingRobot(std::pair<Vector2D, Vector2D>* prevPoseArr)
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	// Detect One with 22.5 Degree (Return -1 if Fails/Detects more than 2)
	// Information: prevPoseArr, poseArr
	int result = -1;
	for (int i = 0; i < OptitrackCommunicator::maxRobotLimit; i++)
	{
		if (prevPoseArr[i].first != OptitrackCommunicator::poseArr[i].first) continue;
		Vector2D calVect = Vector2D::radianToVector(CAL_ROT_DEG * 3.141592 / 180.0);
		if (Vector2D::isSimilar(OptitrackCommunicator::poseArr[i].second - prevPoseArr[i].second, calVect)) {
			if (result != -1) break;
			else result = i;
		}
	}
	if (result == -1) return std::make_pair(result, std::make_pair(Vector2D(), Vector2D()));
	return std::make_pair(result, OptitrackCommunicator::poseArr[result]);
}