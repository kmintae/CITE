/**
 * ProgramState.cpp
 * Purpose: Class w/ Overall Program Information w/ Robot List
 * @author Mintae Kim
 * @author chrisysl (https://kevinthegrey.tistory.com/26)
 */

#include "ProgramState.h"

 // Waiting Buffer
std::queue<Client*> ProgramState::waitingBuffer;

// OptitrackCommunicator
OptitrackCommunicator ProgramState::optitrackCommunicator;

// Variables
int ProgramState::srcBrickLayerIndex = 0;
int ProgramState::dstBrickLayerIndex = 0;
bool ProgramState::isWorking = false; // Current Program State
bool ProgramState::isTerminationActivated = false; // Signal

int ProgramState::connClientNum = 0;

int ProgramState::rotationError = GetPrivateProfileInt("error", "ROTATE_ERROR", 2, "../config/server.ini");

// Mutexes
std::mutex ProgramState::mtx_state;
std::condition_variable ProgramState::cv_state;

std::mutex ProgramState::mtx_connect;
std::mutex ProgramState::mtx_program;

// Robot Lists
int ProgramState::maxClientNum = GetPrivateProfileInt("connection", "MAX_ROBOT_CONNECTED", 2, "../config/server.ini");
Client** ProgramState::clients = new Client * [ProgramState::maxClientNum];

BrickLayerList* ProgramState::brickLayerList= new BrickLayerList(&ProgramState::mtx_state, &ProgramState::cv_state);
Grid* ProgramState::grid = new Grid(&ProgramState::mtx_state, &ProgramState::cv_state);

ProgramState::ProgramState()
{
	for (int i = 0; i < ProgramState::maxClientNum; i++) {
		ProgramState::clients[i] = new Client();
	}
	
	// Lock Acquired
	std::unique_lock<std::mutex> lck(ProgramState::mtx_connect);
	ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
	lck.unlock();
	
	// Multi-Threading
	thread_conn = std::thread(&ProgramState::optitrackConnect, this);
	thread_grid_printing = std::thread(&ProgramState::printGrid, this);
}
ProgramState::~ProgramState()
{
	// Thread Joining
	thread_conn.join();
	for (int i = 0; i < ProgramState::maxClientNum; i++) {
		if (ProgramState::clients[i]->phase == ClientPhase::ACCEPTED) ProgramState::clients[i]->clientThread.join();
		delete ProgramState::clients[i];
	}
	
	// De-Allocation
	delete ProgramState::brickLayerList;
	delete ProgramState::grid;
	delete[] ProgramState::clients;
}

// Executed in Independent Thread
void ProgramState::acceptClient(SOCKET* serverSock)
{
	SOCKADDR_IN tClntAddr = {};
	int iClntSize = sizeof(tClntAddr);
	SOCKET hClient = accept(*serverSock, (SOCKADDR*)&tClntAddr, &iClntSize);

	// Emergency Exit
	// Lock Acquired
	std::unique_lock<std::mutex> lck_program(ProgramState::mtx_program);
	if (ProgramState::isTerminationActivated) {
		closesocket(hClient);
		return;
	}
	lck_program.unlock();

	// Lock Acquired
	std::unique_lock<std::mutex> lck(ProgramState::mtx_connect);

	// Checking Current Client Number
	if (ProgramState::connClientNum == ProgramState::maxClientNum) {
		closesocket(hClient);
		return;
	}

	// Making new Client Object
	Client* newClient = new Client();
	newClient->accept(hClient, tClntAddr);

	// Push newClient into connQueue
	ProgramState::waitingBuffer.push(newClient);
}

void ProgramState::optitrackConnect()
{
	while (true)
	{
		// Emergency Exit
		// Lock Acquired
		std::unique_lock<std::mutex> lck_program(ProgramState::mtx_program);
		if (ProgramState::isTerminationActivated) {
			return;
		}
		lck_program.unlock();

		// Lock Acquired
		std::unique_lock<std::mutex> lck(ProgramState::mtx_connect); // Updating connQueue
		if (ProgramState::brickLayerList->isDone()) {
			break;
		}

		// Assumption: Connection will be held without conflicts incurred by other robots
		while (!ProgramState::waitingBuffer.empty())
		{
			Client* head = ProgramState::waitingBuffer.front();
			
			// Checking Current Client Number
			if (ProgramState::connClientNum < ProgramState::maxClientNum) {
				// Receiving Done
				SOCKET& sock = head->getSocket();
				char buff[MAX_BUFF_SIZE] = { 0, };
				int bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
				if (bytes_recv == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					closesocket(sock);
					delete head;
					break;
				}
				int robotNum = atoi(buff);
				std::pair<Vector2D, Vector2D> initialPos = ProgramState::optitrackCommunicator.getPose(robotNum);

				std::unique_lock<std::mutex> lck2(ProgramState::mtx_state);
				head->connect(robotNum, initialPos);
				ProgramState::clients[robotNum] = head;
				ProgramState::clients[robotNum]->connectedHistory = true;

				ProgramState::connClientNum++;

				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

				// Start Multi-threading
				ProgramState::clients[robotNum]->clientThread = std::thread(&ProgramState::workSession, this, ProgramState::clients[robotNum]);

				// For Tests
				// ProgramState::clients[robotNum]->clientThread = std::thread(&ProgramState::workSession_Test, this, ProgramState::clients[robotNum]);
				
				/* Failed Method: Rotating 22.5 Degree
				bool socketTerminated = false;
				std::pair<int, std::pair<Vector2D, Vector2D>> robotInfo;
				while (true)
				{
					// Try Connection
					std::pair<Vector2D, Vector2D>* prevPoseArr = ProgramState::optitrackCommunicator.getPoseArray();

					// Sending 'PID' Instruction, Rotating 22.5 Degree
					SOCKET& sock = head->getSocket();
					float param[MAX_INST_PARAM] = {};
					int bytes_send = send(sock, Instruction(InstructionType::MOV, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
					if (bytes_send == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
						
						delete[] prevPoseArr;
						socketTerminated = true;
						closesocket(sock);
						delete head;
						break;
					}

					// Receiving Done
					char buff[MAX_BUFF_SIZE] = { 0, };
					int bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
					if (bytes_recv == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
						
						delete[] prevPoseArr;
						socketTerminated = true;
						closesocket(sock);
						delete head;
						break;
					}

					// Determination of Corresponding Robot Number from OptitrackCommunicator
					robotInfo = ProgramState::optitrackCommunicator.getConnectWaitingRobot(prevPoseArr);

					delete[] prevPoseArr;

					if (robotInfo.first == -1) continue;

					// Check if robot_num is already allocated
					bool isUsing = false;
					std::unique_lock<std::mutex> lck2(ProgramState::mtx_state);
					if (ProgramState::clients[robotInfo.first]->phase != ClientPhase::DISCONNECTED) isUsing = true;
					lck2.unlock();

					if (!isUsing) {
						break;
					}
				}

				// Update Array
				if (!socketTerminated)
				{
					std::unique_lock<std::mutex> lck2(ProgramState::mtx_state);
					head->connect(robotInfo.first, robotInfo.second);
					ProgramState::clients[robotInfo.first] = head;
					// Start Multi-threading
					ProgramState::clients[robotInfo.first]->clientThread = std::thread(&ProgramState::workSession, this, ProgramState::clients[robotInfo.first]);

					ProgramState::connClientNum++;
					lck2.unlock();
				}
				else delete head;
				*/
			}
			waitingBuffer.pop();
		}
	}
}

void ProgramState::workSession(Client* client)
{
	while (true)
	{
		// Emergency Exit
		// Lock Acquired
		std::unique_lock<std::mutex> lck_program(ProgramState::mtx_program);
		if (ProgramState::isTerminationActivated) {
			return;
		}
		lck_program.unlock();

		// Implement Workflow
		std::unique_lock<std::mutex> lck(ProgramState::mtx_state, std::defer_lock);

		// 1. Update Position Information
		lck.lock();
		Robot* robot = client->getRobot();
		std::pair<Vector2D, Vector2D> pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
		robot->setPose(pose);
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

		// 2. Classify 
		Brick *srcBrick, *dstBrick;
		int bytes_send, bytes_recv;
		SOCKET sock = client->getSocket();
		std::pair<Vector2D, Vector2D> srcFinalPose, dstFinalPose;
		std::pair<Vector2D, Vector2D> keypoint, finalPose;
		std::pair<Vector2D, Vector2D> initialPose;
		Vector3D srcBrickPos, dstBrickPos;
		float param[MAX_INST_PARAM];
		char buff[MAX_BUFF_SIZE];
		bool isFirst = true, rotate;

		std::string inst;

		switch (robot->phase)
		{
		case RobotPhase::STOP: // 2-1. STOP: Decision of srcBrick
			srcBrick = ProgramState::brickLayerList->getNextSrcBrick(robot, ProgramState::srcBrickLayerIndex, lck);

			// All Jobs Done: Disconnect
			if (srcBrick == NULL) {
				inst = Instruction(InstructionType::DCN, NULL).toString();
				bytes_send = send(sock, inst.c_str(), inst.length(), 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
				}

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			ProgramState::brickLayerList->markAsSelectedBrick(ProgramState::srcBrickLayerIndex, srcBrick);
			srcFinalPose = ProgramState::brickLayerList->getFinalPose(ProgramState::grid, srcBrick, lck);
			robot->markAsMove(srcBrick, srcFinalPose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			break;


		case RobotPhase::MOVING: // 2-2. MOVING: Decision of Path, Saving Keypoint, and sending Instructions (MOV, PID)
			pose = robot->getPose();
			keypoint = robot->getKeypoint();
			finalPose = robot->getFinalPose();

			// Send PID
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;
			param[4] = keypoint.first.x; param[5] = keypoint.first.y; param[6] = keypoint.second.x; param[7] = keypoint.second.y;

			inst = Instruction(InstructionType::PID, param).toString();
			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}
			
			// Update
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);


			if (isFirst) {
				isFirst = false;
				initialPose = pose;
			}
			
			// Comparison with finalPose
			rotate = Vector2D::calculateDistance(initialPose.first, finalPose.first) < ProgramState::rotationError;

			if (Vector2D::isNearest(pose.first, finalPose.first) || rotate)
			{
				if (!rotate || Vector2D::isSimilar(pose.second, finalPose.second)) {
					// Send HLT
					inst = Instruction(InstructionType::HLT, NULL).toString();
					bytes_send = send(sock, inst.c_str(), inst.length(), 0);
					if (bytes_send == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					// Receive DONE/ERROR
					std::fill_n(buff, MAX_BUFF_SIZE, 0);
					bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
					if (bytes_recv == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}

					// Change Phase to Grab
					ProgramState::brickLayerList->markAsGrabbedBrick(robot->getSourceBrick());
					robot->markAsGrab();

					// Update
					pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
					robot->setPose(pose);
					ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

					isFirst = false;
					break;
				}
			}

			// Comparison with keypoint
			rotate = Vector2D::calculateDistance(initialPose.first, keypoint.first) < ProgramState::rotationError;

			if (Vector2D::isNearest(pose.first, keypoint.first) || rotate)
			{
				if (!rotate || Vector2D::isSimilar(pose.second, keypoint.second)) {
					// Send HLT
					inst = Instruction(InstructionType::HLT, NULL).toString();
					bytes_send = send(sock, inst.c_str(), inst.length(), 0);
					if (bytes_send == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					// Receive DONE/ERROR
					std::fill_n(buff, MAX_BUFF_SIZE, 0);
					bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
					if (bytes_recv == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					
					// Pathfinding, Save Keypoint in Robot
					robot->setPath(pathFinding(pose, finalPose, ProgramState::grid, lck, ProgramState::cv_state));
					keypoint = robot->getKeypoint();

					// Update
					pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
					robot->setPose(pose);
					ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
					
					isFirst = false;
					break;
				}
			}

			// MOV or MVL
			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

			if (Vector2D::calculateDistance(initialPose.first, keypoint.first) > ProgramState::rotationError) inst = Instruction(InstructionType::MOV, param).toString();
			else inst = Instruction(InstructionType::MVL, param).toString();

			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Update
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
			
			lck.unlock();
			break;
		
		
		case RobotPhase::GRAB: // 2-3. GRAB: Sending 'GRB' Instruction, and decision of dstBrick
			// 2-3-1. Send GRB, and receive 'DONE'
			// Send GRB
			srcBrick = robot->getSourceBrick();
			srcBrickPos = srcBrick->getPos3D();

			lck.unlock();

			// Send GRB
			param[0] = sqrt(pow(Vector2D::calculateDistance(srcBrickPos.getVect2D(), pose.first), 2.0) + srcBrickPos.z * srcBrickPos.z);
			param[1] = srcBrickPos.z;

			inst = Instruction(InstructionType::GRB, param).toString();
			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				lck.lock();
				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				lck.lock();
				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// 2-3-2. Re-update Information
			lck.lock();
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
			
			// 2-3-3. Deciding dstBrick
			dstBrick = ProgramState::brickLayerList->getNextDstBrick(robot, ProgramState::dstBrickLayerIndex, lck);

			// All Jobs Done: Disconnect
			if (srcBrick == NULL) {
				// FIX

				inst = Instruction(InstructionType::DCN, NULL).toString();
				bytes_send = send(sock, inst.c_str(), inst.length(), 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
				}
				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			ProgramState::brickLayerList->markAsLiftedBrick(robot->getSourceBrick(), ProgramState::dstBrickLayerIndex, dstBrick);
			dstFinalPose = ProgramState::brickLayerList->getFinalPose(ProgramState::grid, dstBrick, lck);
			robot->markAsLift(dstBrick, dstFinalPose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			break;


		case RobotPhase::LIFTING: // 2-4. LIFTING: Decision of Path, Saving Keypoint, and sending Instructions (MOV, PID)
			pose = robot->getPose();
			keypoint = robot->getKeypoint();
			finalPose = robot->getFinalPose();

			// Send PID
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;
			param[4] = keypoint.first.x; param[5] = keypoint.first.y; param[6] = keypoint.second.x; param[7] = keypoint.second.y;

			inst = Instruction(InstructionType::PID, param).toString();
			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Update
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);


			if (isFirst) {
				isFirst = false;
				initialPose = pose;
			}

			// Comparison with finalPose
			rotate = Vector2D::calculateDistance(initialPose.first, finalPose.first) < ProgramState::rotationError;

			if (Vector2D::isNearest(pose.first, finalPose.first) || rotate)
			{
				if (!rotate || Vector2D::isSimilar(pose.second, finalPose.second)) {
					// Send HLT
					inst = Instruction(InstructionType::HLT, NULL).toString();
					bytes_send = send(sock, inst.c_str(), inst.length(), 0);
					if (bytes_send == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					// Receive DONE/ERROR
					std::fill_n(buff, MAX_BUFF_SIZE, 0);
					bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
					if (bytes_recv == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}

					// Change Phase to Release
					ProgramState::brickLayerList->markAsReleasedBrick(robot->getSourceBrick());
					robot->markAsRelease();

					// Update
					pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
					robot->setPose(pose);
					ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

					isFirst = false;
					break;
				}
			}

			// Comparison with keypoint
			rotate = Vector2D::calculateDistance(initialPose.first, keypoint.first) < ProgramState::rotationError;

			if (Vector2D::isNearest(pose.first, keypoint.first) || rotate)
			{
				if (!rotate || Vector2D::isSimilar(pose.second, keypoint.second)) {
					// Send HLT
					inst = Instruction(InstructionType::HLT, NULL).toString();
					bytes_send = send(sock, inst.c_str(), inst.length(), 0);
					if (bytes_send == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					// Receive DONE/ERROR
					std::fill_n(buff, MAX_BUFF_SIZE, 0);
					bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
					if (bytes_recv == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}

					// Pathfinding, Save Keypoint in Robot
					robot->setPath(pathFinding(pose, finalPose, ProgramState::grid, lck, ProgramState::cv_state));
					keypoint = robot->getKeypoint();

					// Update
					pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
					robot->setPose(pose);
					ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

					isFirst = false;
					break;
				}
			}

			// MOV or MVL
			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

			if (Vector2D::calculateDistance(initialPose.first, keypoint.first) > ProgramState::rotationError) inst = Instruction(InstructionType::MOV, param).toString();
			else inst = Instruction(InstructionType::MVL, param).toString();

			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Update
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			break;
			

		case RobotPhase::RELEASE: // 2-5. Release: Send 'RLZ' Instruction, and setting phase to STOP
			// 2-5-1. Send RLZ, and receive 'DONE'
			// Send RLZ
			dstBrick = robot->getDestinationBrick();
			dstBrickPos = dstBrick->getPos3D();

			lck.unlock();
			// Send RLZ
			param[0] = sqrt(pow(Vector2D::calculateDistance(dstBrickPos.getVect2D(), pose.first), 2.0) + dstBrickPos.z * dstBrickPos.z);
			param[1] = dstBrickPos.z;

			inst = Instruction(InstructionType::RLZ, param).toString();
			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				lck.lock();
				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}
			
			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				lck.lock();
				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}
			
			// 2-5-2. Re-update Information
			lck.lock();
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			
			// 2-5-3. Change phase to STOP
			ProgramState::brickLayerList->markAsDone(ProgramState::srcBrickLayerIndex, robot->getSourceBrick(), 
				ProgramState::dstBrickLayerIndex, robot->getDestinationBrick());
			robot->markAsStop();
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
			break;
		}
		// Sleep for 0.5s
		Sleep(500);
	}
}

// Print Grid
void ProgramState::printGrid()
{
	/*
	while (true) {
		std::unique_lock<std::mutex> lck(ProgramState::mtx_state);

		system("cls");
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		bool** grid_bool = ProgramState::grid->getGrid();

		for (int i = ProgramState::grid->y - 1; i >= 0; i--) {
			for (int j = 0; j < ProgramState::grid->x; j++) {
				if (grid_bool[i][j]) printf("\u25A0");
				else printf("\u25A1");
			}
			printf("\n");
		}
		lck.unlock();
		// Sleep for 0.1s
		Sleep(100);
	}
	*/
}

// Emergency Stop
void ProgramState::makeStop()
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(ProgramState::mtx_program); // Updating connQueue

	ProgramState::isTerminationActivated = true;
}







// For Tests
void ProgramState::workSession_Test(Client* client)
{
	Sleep(1500);
	// Emergency Exit
	// Lock Acquired
	std::unique_lock<std::mutex> lck_program(ProgramState::mtx_program);
	if (ProgramState::isTerminationActivated) {
		return;
	}
	lck_program.unlock();

	// Implement Workflow
	std::unique_lock<std::mutex> lck(ProgramState::mtx_state, std::defer_lock);

	// 1. Update Position Information
	lck.lock();
	Robot* robot = client->getRobot();
	std::pair<Vector2D, Vector2D> pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
	robot->setPose(pose);
	ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

	// 2. Brick
	Brick* brick = new Brick();
	brick->setPos(600.0, 600.0, 13.5);
	brick->setDir(0, 1, 0);
	std::pair<Vector2D, Vector2D> finalPose = ProgramState::brickLayerList->getFinalPose(ProgramState::grid, brick, lck);
	robot->markAsMove(brick, finalPose);

	// 3. Three KeyPoints
	std::pair<Vector2D, Vector2D> keypoint1 = std::make_pair(Vector2D(finalPose.first.x, pose.first.y), pose.second);
	Vector2D newDir = Vector2D(0, -1);
	// std::pair<Vector2D, Vector2D> keypoint2 = std::make_pair(Vector2D(600, pose.first.y), newDir);
	// std::pair<Vector2D, Vector2D> keypoint3 = std::make_pair(Vector2D(600, 1800), newDir);

	int bytes_send, bytes_recv;
	SOCKET sock = client->getSocket();
	float param[MAX_INST_PARAM];
	char buff[MAX_BUFF_SIZE] = { 0, };

	// Send SET
	//param[0] =0.03; param[1] = 0.036; param[2] = 0.0004; param[3] = 10;
	//param[4] = 0.15; param[5] = 1.3 ; param[6] = 0.65; param[7] = 5; //0.0

	param[0] =0.09; param[1] = 0.036; param[2] = 0.0006; param[3] = 10;
	param[4] = 0.3; param[5] = 1.0; param[6] = 0.65; param[7] = 5; //0.0021
	lck.unlock();
	std::string inst;
	
	inst = Instruction(InstructionType::SET, param).toString();
	bytes_send = send(sock, inst.c_str(), inst.length(), 0);
	if (bytes_send == SOCKET_ERROR) {
		fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

		lck.lock();
		delete client; // closeSocket + De-allocation
		client = new Client();
		client->connectedHistory = true;
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		ProgramState::connClientNum--;
		return;
	}

	// Receive DONE/ERROR
	std::fill_n(buff, MAX_BUFF_SIZE, 0);
	bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
	if (bytes_recv == SOCKET_ERROR) {
		fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

		lck.lock();
		delete client; // closeSocket + De-allocation
		client = new Client();
		client->connectedHistory = true;
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		ProgramState::connClientNum--;
		return;
	}
	
	// Send PID
	lck.lock();
	pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
	robot->setPose(pose);
	ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

	param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;
	param[4] = keypoint1.first.x; param[5] = keypoint1.first.y; param[6] = keypoint1.second.x; param[7] = keypoint1.second.y;
	lck.unlock();
	
	bool isFirst = true;
	std::pair<Vector2D, Vector2D> initialPose;

	std::pair<Vector2D, Vector2D> keypointArr[4] = {
		keypoint1,
		std::make_pair(Vector2D(finalPose.first.x, pose.first.y), newDir ),
		std::make_pair(Vector2D(finalPose.first.x, finalPose.first.y), newDir ),
		std::make_pair(Vector2D(finalPose.first.x, finalPose.first.y), finalPose.second)
	};
	
	printf("FinalPose : %s, %s\n", finalPose.first.toString().c_str(), finalPose.second.toString().c_str());

	for (int i = 0; i < 4; i++) {
		std::pair<Vector2D, Vector2D> keypoint = keypointArr[i];
		isFirst = true;

		if (i < 4) {
			// Send PID
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;
			param[4] = keypoint.first.x; param[5] = keypoint.first.y; param[6] = keypoint.second.x; param[7] = keypoint.second.y;

			inst = Instruction(InstructionType::PID, param).toString();
			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}

			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}
		}
		else {
			// 원래는 Brick 관련 설정
		}

		while (true)
		{
			lck.lock();
			robot = client->getRobot();
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);

			printf("[Init] Pos: %s , Dir: %s\n", initialPose.first.toString().c_str(), initialPose.second.toString().c_str());
			printf("[pose] Pos: %s , Dir: %s\n", pose.first.toString().c_str(), pose.second.toString().c_str());
			printf("[key %d] Pos: %s , Dir: %s\n", i, keypoint.first.toString().c_str(), keypoint.second.toString().c_str());

			if (isFirst) {
				isFirst = false;
				initialPose = pose;
			}
			bool rotate = Vector2D::calculateDistance(initialPose.first, keypoint.first) < ProgramState::rotationError;

			if (Vector2D::isNearest(pose.first, keypoint.first)||rotate)
			{
				if ( !rotate|| Vector2D::isSimilar(pose.second, keypoint.second)) {
					// Send HLT
					inst = Instruction(InstructionType::HLT, NULL).toString();
					bytes_send = send(sock, inst.c_str(), inst.length(), 0);
					if (bytes_send == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					// Receive DONE/ERROR
					std::fill_n(buff, MAX_BUFF_SIZE, 0);
					bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
					if (bytes_recv == SOCKET_ERROR) {
						fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

						delete client; // closeSocket + De-allocation
						client = new Client();
						client->connectedHistory = true;
						ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
						ProgramState::connClientNum--;
						return;
					}
					
					ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
					lck.unlock();
					break;
				}
			}

			// MOV or MVL
			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

			if (Vector2D::calculateDistance(initialPose.first, keypoint.first) > ProgramState::rotationError) inst = Instruction(InstructionType::MOV, param).toString();
			else inst = Instruction(InstructionType::MVL, param).toString();

			bytes_send = send(sock, inst.c_str(), inst.length(), 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}
			// Receive DONE/ERROR
			std::fill_n(buff, MAX_BUFF_SIZE, 0);
			bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
			if (bytes_recv == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				client->connectedHistory = true;
				ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
				ProgramState::connClientNum--;
				return;
			}
			ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
			lck.unlock();
			Sleep(10);
		}
		printf("SLEEP\n");
		Sleep(500);
	}

	// Send GRB
	Vector3D brickPos = brick->getPos3D();

	lck.lock();
	pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
	robot->setPose(pose);
	ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
	param[0] = Vector2D::calculateDistance(brickPos.getVect2D(), pose.first);
	param[1] = brickPos.z;
	lck.unlock();

	inst = Instruction(InstructionType::GRB, param).toString();
	bytes_send = send(sock, inst.c_str(), inst.length(), 0);
	if (bytes_send == SOCKET_ERROR) {
		fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

		lck.lock();
		delete client; // closeSocket + De-allocation
		client = new Client();
		client->connectedHistory = true;
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		ProgramState::connClientNum--;
		return;
	}

	// Receive DONE/ERROR
	std::fill_n(buff, MAX_BUFF_SIZE, 0);
	bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
	if (bytes_recv == SOCKET_ERROR) {
		fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

		lck.lock();
		delete client; // closeSocket + De-allocation
		client = new Client();
		client->connectedHistory = true;
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		ProgramState::connClientNum--;
		return;
	}

	Sleep(1000);

	lck.lock();
	pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
	robot->setPose(pose);
	ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
	param[0] = sqrt(pow(Vector2D::calculateDistance(brickPos.getVect2D(), pose.first), 2.0) + brickPos.z * brickPos.z);
	param[1] = brickPos.z;
	lck.unlock();

	inst = Instruction(InstructionType::RLZ, param).toString();
	bytes_send = send(sock, inst.c_str(), inst.length(), 0);
	if (bytes_send == SOCKET_ERROR) {
		fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

		lck.lock();
		delete client; // closeSocket + De-allocation
		client = new Client();
		client->connectedHistory = true;
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		ProgramState::connClientNum--;
		return;
	}

	// Receive DONE/ERROR
	std::fill_n(buff, MAX_BUFF_SIZE, 0);
	bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
	if (bytes_recv == SOCKET_ERROR) {
		fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

		lck.lock();
		delete client; // closeSocket + De-allocation
		client = new Client();
		client->connectedHistory = true;
		ProgramState::grid->repaint(ProgramState::brickLayerList, &ProgramState::optitrackCommunicator, ProgramState::clients);
		ProgramState::connClientNum--;
		return;
	}

	printf("End\n");
}