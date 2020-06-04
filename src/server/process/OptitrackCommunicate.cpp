/**
 * OptitrackCommunicate.cpp
 * Purpose: Optitrack Communication via UDP Protocol
 * @author Mintae Kim
 */

#include "OptitrackCommunicate.h"

void UDPSocket(WSAData &wsaData, SOCKET &udpSocket)
{
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	udpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	int port;
	port = GetPrivateProfileInt("connection", "UDP_PORT", -1, "../config/server.ini");
	if (port == -1) {
		fprintf(stderr, "config/server.ini Not Existing. Terminating...\n");
		return;
	}

	SOCKADDR_IN servAddr = {};
	servAddr.sin_family = AF_INET;
	servAddr.sin_port = htons(port);
	servAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // Localhost

	if (connect(udpSocket, (SOCKADDR*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR)
	{
		fprintf(stderr, "UDP Connect() Error\n");
		closesocket(udpSocket);
		return;
	}
	char request[MAX_UDP_BUFF_SIZE] = "Request";
	if (send(udpSocket, request, MAX_UDP_BUFF_SIZE, 0) == SOCKET_ERROR) {
		fprintf(stderr, "UDP Connect() Error\n");
		closesocket(udpSocket);
		return;
	}
}

int fetchOptitrackData(SOCKET& udpSocket, std::string &data)
{
	char buff[MAX_UDP_BUFF_SIZE];
	if (recv(udpSocket, buff, MAX_UDP_BUFF_SIZE, 0) == SOCKET_ERROR) return SOCKET_ERROR;
	data = std::string(buff);
	return 0;
}