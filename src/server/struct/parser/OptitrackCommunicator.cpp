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

OptitrackCommunicator::OptitrackCommunicator() : communicator()
{
	int maxRobotLimit = GetPrivateProfileInt("connection", "MAX_ROBOT_CONNECTED", 2, "../config/server.ini");

	poseArr = new std::pair<Vector2D, Vector2D>[maxRobotLimit];

	// Socket Initiation
	UDPSocket(OptitrackCommunicator::udpSocket);
	
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

	delete[] poseArr;
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
		std::string rawData = fetchOptitrackData(OptitrackCommunicator::udpSocket);
		
		// Update Array
		updateArray(rawData);

		// Sleep for 0.5s
		Sleep(500);
	}
}
void OptitrackCommunicator::updateArray(std::string rawData)
{
	// Parse rawData & Update Array
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);
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
			std::getline(ss2, ID_s, ',');
			ID = stoi(ID_s);

			// X
			std::getline(ss2, X_s, ',');
			X = stof(X_s);

			// Y
			std::getline(ss2, Y_s, ',');
			Y = stof(Y_s);

			// Theta
			std::getline(ss2, theta_s, ',');
			theta = stof(theta_s);
			
			poseArr[ID].first.x = X;
			poseArr[ID].first.y = Y;

			Vector2D dir = Vector2D::radianToVector(theta);
			poseArr[ID].second = dir;
		}
	}
}

// Don't Forget to delete after using get Pos/Dir Array()
std::pair<Vector2D, Vector2D>* OptitrackCommunicator::getPoseArray()
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	std::pair<Vector2D, Vector2D>* newPoseArr = new std::pair<Vector2D, Vector2D>[maxRobotLimit];
	for (int i = 0; i < maxRobotLimit; i++) newPoseArr[i] = poseArr[i];

	return newPoseArr;
}

std::pair<Vector2D, Vector2D> OptitrackCommunicator::getPose(int robotNum)
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	return poseArr[robotNum];
}

std::pair<int, std::pair<Vector2D, Vector2D>> OptitrackCommunicator::getConnectWaitingRobot(std::pair<Vector2D, Vector2D>* prevPoseArr)
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(OptitrackCommunicator::mtx);

	// Detect One with 22.5 Degree (Return -1 if Fails/Detects more than 2)
	// Information: prevPoseArr, poseArr
	int result = -1;
	for (int i = 0; i < maxRobotLimit; i++)
	{
		if (prevPoseArr[i].first != poseArr[i].first) continue;
		if (Vector2D::isSimilar(poseArr[i].second - prevPoseArr[i].second, Vector2D::radianToVector(CAL_ROT_DEG * 3.141592 / 180.0))) {
			if (result != -1) break;
			else result = i;
		}
	}
	if (result == -1) return std::make_pair(result, std::make_pair(Vector2D(), Vector2D()));
	return std::make_pair(result, poseArr[result]);
}