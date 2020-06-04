/**
 * OptitrackCommunicate.h
 * Purpose: Optitrack Communication via UDP Protocol
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <stdio.h>
#include <string>
#include <WinSock2.h>
#include <Windows.h>

#pragma comment(lib,"ws2_32")

#define MAX_UDP_BUFF_SIZE 512

void UDPSocket(WSAData& wsaData, SOCKET& udpSocket);

int fetchOptitrackData(SOCKET& udpSocket, std::string& data);