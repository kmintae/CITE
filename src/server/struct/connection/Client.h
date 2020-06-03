/**
 * Client.h
 * Purpose: Encapsulates Client Information (w/ Robot Object)
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <WinSock2.h>
#include <thread>

#include "../basic_blocks/Phase.h"
#include "../basic_blocks/Robot.h"

#include "../vector/Position.h"

class Client
{
private:
	Robot* robot;
	SOCKET sock;
	SOCKADDR_IN sock_in;

public:
	std::thread clientThread;
	ClientPhase phase;

	Client();
	
	~Client();

	void accept(SOCKET& sock, SOCKADDR_IN& sock_in);
	void connect(int robotNum, std::pair<Position2D, Direction2D> initialPose);
	
	SOCKET& getSocket();
	Robot* getRobot();
};