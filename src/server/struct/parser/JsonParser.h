/**
 * JsonParser.h
 * Purpose: Json Parser w/ nhloman JSON PARSER
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <vector>
#include <fstream>
#include <Windows.h>
#include <algorithm>
#include <cmath>

#include "../basic_blocks/Brick.h"
#include "../basic_blocks/BrickLayer.h"

#include "../vector/Position.h"
#include "../vector/Direction.h"

class JsonParser
{
private:
	std::vector<BrickLayer*> srcBrickLayerList, dstBrickLayerList;
public:
	JsonParser();

	// Just Copy
	std::vector<BrickLayer*> getSrcBrickLayerList();
	std::vector<BrickLayer*> getDstBrickLayerList();
};