#include "server.h"

int main()
{
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	// TODO: Implement
	ProgramState programState;

	serverSocket(&programState);

	WSACleanup();
	return 0;
}