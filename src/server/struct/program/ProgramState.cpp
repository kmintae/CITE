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
bool ProgramState::isTerminationActivated; // Signal

int ProgramState::connClientNum = 0;

ProgramState::ProgramState() : thread_conn()
{
	brickLayerList = new BrickLayerList(&mtx_state, &cv_state);
	grid = new Grid(&mtx_state, &cv_state);

	maxClientNum = GetPrivateProfileInt("connection", "MAX_ROBOT_CONNECTED", 2, "../config/server.ini");

	clients = new Client*[ProgramState::maxClientNum];
	for (int i = 0; i < ProgramState::maxClientNum; i++) {
		clients[i] = new Client();
	}

	// Multi-Threading
	thread_conn = std::thread(&ProgramState::optitrackConnect, this);
}
ProgramState::~ProgramState()
{
	// Thread Joining
	thread_conn.join();
	for (int i = 0; i < ProgramState::maxClientNum; i++) {
		if (clients[i]->phase == ClientPhase::ACCEPTED) clients[i]->clientThread.join();
		delete clients[i];
	}
	
	// De-Allocation
	delete brickLayerList;
	delete grid;
	delete[] clients;
}

// Executed in Independent Thread
void ProgramState::acceptClient(SOCKET* serverSock)
{
	SOCKADDR_IN tClntAddr = {};
	int iClntSize = sizeof(tClntAddr);
	SOCKET hClient = accept(*serverSock, (SOCKADDR*)&tClntAddr, &iClntSize);

	// Emergency Exit
	// Lock Acquired
	std::unique_lock<std::mutex> lck_program(mtx_program);
	if (ProgramState::isTerminationActivated) {
		closesocket(hClient);
		return;
	}
	lck_program.unlock();

	// Lock Acquired
	std::unique_lock<std::mutex> lck(mtx_connect);

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
		std::unique_lock<std::mutex> lck_program(mtx_program);
		if (ProgramState::isTerminationActivated) {
			return;
		}
		lck_program.unlock();

		// Lock Acquired
		std::unique_lock<std::mutex> lck(mtx_connect); // Updating connQueue

		if (brickLayerList->isDone()) {
			break;
		}

		// Assumption: Connection will be held without conflicts incurred by other robots
		while (!waitingBuffer.empty())
		{
			Client* head = waitingBuffer.front();
			
			// Checking Current Client Number
			if (ProgramState::connClientNum < ProgramState::maxClientNum) {
				bool socketTerminated = false;
				std::pair<int, std::pair<Vector2D, Vector2D>> robotInfo;
				while (true)
				{
					// Try Connection
					std::pair<Vector2D, Vector2D>* prevPoseArr = ProgramState::optitrackCommunicator.getPoseArray();

					// Sending 'CAL' Instruction
					SOCKET& sock = head->getSocket();
					int bytes_send = send(sock, Instruction(InstructionType::CAL, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
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
					if (bytes_send == SOCKET_ERROR) {
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
					std::unique_lock<std::mutex> lck2(mtx_state);
					if (clients[robotInfo.first]->phase != ClientPhase::DISCONNECTED) isUsing = true;
					lck2.unlock();

					if (!isUsing) {
						break;
					}
				}

				// Update Array
				if (!socketTerminated)
				{
					std::unique_lock<std::mutex> lck2(mtx_state);
					head->connect(robotInfo.first, robotInfo.second);
					clients[robotInfo.first] = head;
					// Start Multi-threading
					clients[robotInfo.first]->clientThread = std::thread(&ProgramState::workSession, this, clients[robotInfo.first]);

					ProgramState::connClientNum++;
					lck2.unlock();
				}
				else delete head;
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
		std::unique_lock<std::mutex> lck_program(mtx_program);
		if (ProgramState::isTerminationActivated) {
			return;
		}
		lck_program.unlock();

		// Implement Workflow
		std::unique_lock<std::mutex> lck(mtx_state, std::defer_lock);

		// 1. Update Position Information
		lck.lock();
		Robot* robot = client->getRobot();
		std::pair<Vector2D, Vector2D> pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
		robot->setPose(pose);
		grid->repaint(brickLayerList, &optitrackCommunicator, clients);

		// 2. Classify 
		Brick *srcBrick, *dstBrick;
		int bytes_send, bytes_recv;
		SOCKET sock;
		std::pair<Vector2D, Vector2D> srcFinalPose, dstFinalPose;
		std::pair<Vector2D, Vector2D> keypoint, finalPose;
		Vector3D srcBrickPos, dstBrickPos;
		float param[MAX_INST_PARAM];
		char buff[MAX_BUFF_SIZE];

		switch (robot->phase)
		{
		case RobotPhase::STOP: // 2-1. STOP: Decision of srcBrick
			srcBrick = brickLayerList->getNextSrcBrick(robot, ProgramState::srcBrickLayerIndex, lck);

			// All Jobs Done: Disconnect
			if (srcBrick == NULL) {
				sock = client->getSocket();
				bytes_send = send(sock, Instruction(InstructionType::DCN, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
				}

				delete client; // closeSocket + De-allocation
				client = new Client();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				ProgramState::connClientNum--;
				return;
			}
			srcFinalPose = brickLayerList->getFinalPose(grid, srcBrick, lck);
			robot->markAsMove(srcBrick, srcFinalPose);
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);
			break;


		case RobotPhase::MOVING: // 2-2. MOVING: Decision of Path, Saving Keypoint, and sending Instructions (MOV, PID)
			pose = robot->getPose();
			keypoint = robot->getKeypoint();
			finalPose = robot->getFinalPose();

			// 2-2-1. Check if Current Pose == Final Pose: Send HLT, change phase to GRAB, and break
			if (Vector2D::isNearest(pose.first, finalPose.first) && Vector2D::isSimilar(pose.second, finalPose.second)) {
				// Send HLT
				sock = client->getSocket();
				bytes_send = send(sock, Instruction(InstructionType::HLT, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
					
					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}
				// Change Phase to Grab
				robot->markAsGrab();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				break;
			}
			// 2-2-1-Extra. Check if Dist (Current Pose_Pos, Final Pose_Pos) <= GRID_SIZE: Send MVL Instruction
			if (Vector2D::isNear(pose.first, finalPose.first)) {
				param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

				bytes_send = send(sock, Instruction(InstructionType::MVL, param).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				break;
			}

			// 2-2-2. Check if Current Pose == Keypoint: Send HLT, Pathfinding, Send PID, Receive DONE/ERROR, and break
			if (Vector2D::isNearest(pose.first, keypoint.first) && Vector2D::isSimilar(pose.second, keypoint.second)) { // Current Pose == Keypoint
				// Send HLT
				sock = client->getSocket();
				bytes_send = send(sock, Instruction(InstructionType::HLT, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}

				// Pathfinding, Save Keypoint in Robot
				robot->setPath(pathFinding(pose, finalPose, grid, lck, cv_state));
				keypoint = robot->getKeypoint();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);

				// Send PID
				param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;
				param[4] = keypoint.first.x; param[5] = keypoint.first.y; param[6] = keypoint.second.x; param[7] = keypoint.second.y;

				bytes_send = send(sock, Instruction(InstructionType::PID, param).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					lck.lock();
					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					return;
				}

				// PID RTT is ignorable
				// Receive DONE/ERROR
				std::fill_n(buff, MAX_BUFF_SIZE, 0);
				int bytes_recv = recv(sock, buff, MAX_BUFF_SIZE, 0);
				if (bytes_recv == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					lck.lock();
					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}
				if (buff == "ERROR") {
					fprintf(stderr, "PID Initialization Failed\n");
				}

				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				break;
			}
			
			// 2-2-3. Send 'MOV' Instruction
			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

			bytes_send = send(sock, Instruction(InstructionType::MOV, param).toString().c_str(), MAX_BUFF_SIZE, 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				ProgramState::connClientNum--;
				return;
			}
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);
			break;
		
		
		case RobotPhase::GRAB: // 2-3. GRAB: Sending 'GRB' Instruction, and decision of dstBrick
			// 2-3-1. Send GRB, and receive 'DONE'
			// Send GRB
			srcBrick = robot->getSourceBrick();
			srcBrickPos = srcBrick->getPos3D();

			lck.unlock();

			// Send GRB
			param[0] = srcBrickPos.x; param[1] = srcBrickPos.y; param[2] = srcBrickPos.z;

			bytes_send = send(sock, Instruction(InstructionType::GRB, param).toString().c_str(), MAX_BUFF_SIZE, 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				lck.lock();
				delete client; // closeSocket + De-allocation
				client = new Client();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
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
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				ProgramState::connClientNum--;
				return;
			}

			// 2-3-2. Re-update Information
			lck.lock();
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);
			
			// 2-3-3. Deciding dstBrick
			dstBrick = brickLayerList->getNextDstBrick(robot, ProgramState::dstBrickLayerIndex, lck);

			// All Jobs Done: Disconnect
			if (srcBrick == NULL) {
				sock = client->getSocket();
				bytes_send = send(sock, Instruction(InstructionType::DCN, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");
				}
				delete client; // closeSocket + De-allocation
				client = new Client();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				ProgramState::connClientNum--;
				return;
			}
			dstFinalPose = brickLayerList->getFinalPose(grid, dstBrick, lck);
			robot->markAsLift(dstBrick, dstFinalPose);
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);
			break;


		case RobotPhase::LIFTING: // 2-4. LIFTING: Decision of Path, Saving Keypoint, and sending Instructions (MOV, PID)
			pose = robot->getPose();
			keypoint = robot->getKeypoint();
			finalPose = robot->getFinalPose();

			// 2-4-1. Check if Current Pose == Final Pose: Send HLT, and change phase to RLZ
			if (Vector2D::isNearest(pose.first, finalPose.first) && Vector2D::isSimilar(pose.second, finalPose.second)) { // Current Pose == Final Pose
				// Send HLT
				sock = client->getSocket();
				bytes_send = send(sock, Instruction(InstructionType::HLT, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}

				// Change Phase to Release
				robot->markAsRelease();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				break;
			}

			// 2-4-1-Extra. Check if Dist (Current Pose_Pos, Final Pose_Pos) <= GRID_SIZE: Send MVL Instruction
			if (Vector2D::isNear(pose.first, finalPose.first)) {
				param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

				bytes_send = send(sock, Instruction(InstructionType::MVL, param).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				break;
			}

			// 2-4-2. Check if Current Pose == Keypoint: Send HLT, Send PID, Receive DONE/ERROR
			if (Vector2D::isNearest(pose.first, keypoint.first) && Vector2D::isSimilar(pose.second, keypoint.second)) { // Current Pose == Keypoint
				// Send HLT
				sock = client->getSocket();
				bytes_send = send(sock, Instruction(InstructionType::HLT, NULL).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}

				// Pathfinding, Save Keypoint in Robot
				robot->setPath(pathFinding(pose, finalPose, grid, lck, cv_state));
				keypoint = robot->getKeypoint();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);

				// PID RTT is ignorable
				// Send PID
				param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;
				param[4] = keypoint.first.x; param[5] = keypoint.first.y; param[6] = keypoint.second.x; param[7] = keypoint.second.y;

				bytes_send = send(sock, Instruction(InstructionType::PID, param).toString().c_str(), MAX_BUFF_SIZE, 0);
				if (bytes_send == SOCKET_ERROR) {
					fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

					delete client; // closeSocket + De-allocation
					client = new Client();
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
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
					grid->repaint(brickLayerList, &optitrackCommunicator, clients);
					ProgramState::connClientNum--;
					return;
				}
				if (buff == "ERROR") {
					fprintf(stderr, "PID Initialization Failed\n");
				}

				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				break;
			}

			// 2-4-3. Send 'MOV' Instruction
			param[0] = pose.first.x; param[1] = pose.first.y; param[2] = pose.second.x; param[3] = pose.second.y;

			bytes_send = send(sock, Instruction(InstructionType::MOV, param).toString().c_str(), MAX_BUFF_SIZE, 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				delete client; // closeSocket + De-allocation
				client = new Client();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				ProgramState::connClientNum--;
				return;
			}
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);
			break;


		case RobotPhase::RELEASE: // 2-5. Release: Send 'RLZ' Instruction, and setting phase to STOP
			// 2-5-1. Send RLZ, and receive 'DONE'
			// Send RLZ
			dstBrick = robot->getDestinationBrick();
			dstBrickPos = dstBrick->getPos3D();

			param[0] = dstBrickPos.x; param[1] = dstBrickPos.y; param[2] = dstBrickPos.z;
			lck.unlock();

			bytes_send = send(sock, Instruction(InstructionType::RLZ, param).toString().c_str(), MAX_BUFF_SIZE, 0);
			if (bytes_send == SOCKET_ERROR) {
				fprintf(stderr, "Send Failed, Termination of Socket Connection\n");

				lck.lock();
				delete client; // closeSocket + De-allocation
				client = new Client();
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
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
				grid->repaint(brickLayerList, &optitrackCommunicator, clients);
				ProgramState::connClientNum--;
				return;
			}
			
			// 2-5-2. Re-update Information
			lck.lock();
			pose = ProgramState::optitrackCommunicator.getPose(robot->getRobotNum());
			robot->setPose(pose);
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);

			// 2-5-3. Change phase to STOP
			robot->markAsStop();
			grid->repaint(brickLayerList, &optitrackCommunicator, clients);
			break;
		}
	}
}

// Emergency Stop
void ProgramState::makeStop()
{
	// Lock Acquired
	std::unique_lock<std::mutex> lck(mtx_program); // Updating connQueue

	ProgramState::isTerminationActivated = true;
}