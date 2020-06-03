/**
 * Socket.h
 * Purpose: Client Socket Implementation
 * @author Mintae Kim
 */

 // Reference: https://blog.naver.com/PostView.nhn?blogId=dlghks44&logNo=221262048861&parentCategoryNo=&categoryNo=21&viewDate=&isShowPopularPosts=true&from=search

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <stdio.h> 
#include <string>
#include <Winsock2.h>
#include <Windows.h>

#include "../struct/program/ProgramState.h"

void serverSocket(ProgramState *programState);
