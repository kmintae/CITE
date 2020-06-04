/**
 * ProgramState.h
 * Purpose: Class w/ Overall Program Information w/ Robot List
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include <utility>
#include <stdio.h>
#include <WinSock2.h>
#include <Windows.h>

#include "../../process/PathFinding.h"

#include "../basic_blocks/Instruction.h"
#include "../basic_blocks/Phase.h"
#include "../basic_blocks/Brick.h"
#include "../basic_blocks/BrickLayer.h"
#include "../basic_blocks/Robot.h"

#include "../connection/Client.h"

#include "../parser/OptitrackCommunicator.h"

#include "../states/BrickLayerList.h"
#include "../states/Grid.h"

#include "../vector/Vector.h"

#define MAX_BUFF_SIZE 512

class ProgramState
{
private:
	// Robot Lists
	Client** clients;

	// Waiting Buffer
	static std::queue<Client*> waitingBuffer;

	// OptitrackCommunicator
	static OptitrackCommunicator optitrackCommunicator;

	// Status: BrickListManager, Grid
	BrickLayerList* brickLayerList;
	Grid* grid;

	// Mutex & Condition Variable
	std::mutex mtx_state;
	std::condition_variable cv_state;
	
	std::mutex mtx_connect;
	std::mutex mtx_program;

	// Thread
	std::thread thread_conn;

	// Variables
	static int srcBrickLayerIndex, dstBrickLayerIndex;
	static bool isWorking; // Current Program State
	static bool isTerminationActivated; // Signal
	
	static int connClientNum;
	int maxClientNum;

public:
	ProgramState();
	~ProgramState();

	// Related to Connection
	void acceptClient(SOCKET* serverSock); // Executed in Independent Thread
	void optitrackConnect(); // Executed in Independent Thread

	void workSession(Client *client); // Executed in independent thread (One Thread per Connected Client)

	// Emergency Stop
	void makeStop();
};