/**
 * Client.cpp
 * Purpose: Encapsulates Client Information (w/ Robot Object)
 * @author Mintae Kim
 */

#include "Client.h"

Client::Client()
{
	robot = NULL;

	phase = ClientPhase::DISCONNECTED;
}

Client::~Client()
{
	if (robot != NULL) delete robot;
	closesocket(sock);
}

void Client::accept(SOCKET& sock, SOCKADDR_IN& sock_in)
{
	this->sock = sock;
	this->sock_in = sock_in;

	phase = ClientPhase::ACCEPTED;
}
void Client::connect(int robotNum, std::pair<Position2D, Direction2D> initialPose)
{
	robot = new Robot(robotNum, initialPose);

	phase = ClientPhase::CONNECTED;
}

SOCKET& Client::getSocket()
{
	return sock;
}
Robot* Client::getRobot()
{
	return robot;
}