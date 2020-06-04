/**
 * JsonParser.cpp
 * Purpose: Json Parser w/ nhloman JSON PARSER
 * @author Mintae Kim
 */

// Referencing: https://github.com/nlohmann/json
// Referencing: https://blessingdev.wordpress.com/2017/09/26/visual-studio%EC%97%90-%EC%99%B8%EB%B6%80-%EB%9D%BC%EC%9D%B4%EB%B8%8C%EB%9F%AC%EB%A6%AC-%EC%B6%94%EA%B0%80%ED%95%98%EA%B8%B0/

#include <nlohmann/json.hpp>

#include "JsonParser.h"

using json = nlohmann::json;

JsonParser::JsonParser()
{
	// Json Parsing
	std::ifstream json_stream("../../../data/rhino_200603.json");
	json j;

	json_stream >> j;

	// Declaration of Vector
	std::vector<Brick*> srcBricks, dstBricks;
	Brick* brick;
	for (json::iterator it = j["bricks"].begin(); it != j["bricks"].end(); ++it) {
		brick = new Brick();

		brick->setPos((*it)["point"][0], (*it)["point"][1], (*it)["point"][2]);
		brick->setDir((*it)["dir"][0], (*it)["dir"][1], (*it)["dir"][2]);

		if ((*it)["src"] == true)
			srcBricks.push_back(brick);
		else
			dstBricks.push_back(brick);
	}

	BrickLayer* brickLayer = new BrickLayer();
	float recentZ;
	std::sort(srcBricks.begin(), srcBricks.end(), [](Brick* a, Brick* b) {return *a < *b;  });
	std::sort(dstBricks.begin(), dstBricks.end(), [](Brick* a, Brick* b) {return *a > *b;  });

	char buf[512] = { 0. };
	GetPrivateProfileString("brick", "HEIGHT_MM", "-1", buf, 512, "../config/server.ini");
	float brickHeight = atof(buf) - 0.1;

	if (srcBricks.size() > 0) {
		brickLayer->addBrick(srcBricks[0]);
		recentZ = srcBricks[0]->getPos3D().z;

		for (int i = 1; i < srcBricks.size(); i++) {
			if (abs(recentZ - srcBricks[i]->getPos3D().z) > brickHeight) {
				srcBrickLayerList.push_back(brickLayer);
				recentZ = srcBricks[i]->getPos3D().z;
				brickLayer = new BrickLayer();
			}
			brickLayer->addBrick(srcBricks[i]);
		}
		srcBrickLayerList.push_back(brickLayer);
		brickLayer = new BrickLayer();
	}
	
	if (dstBricks.size() > 0) {
		brickLayer->addBrick(dstBricks[0]);
		recentZ = dstBricks[0]->getPos3D().z;

		for (int i = 1; i < dstBricks.size(); i++) {
			if (abs(recentZ - dstBricks[i]->getPos3D().z) > brickHeight) {
				dstBrickLayerList.push_back(brickLayer);
				recentZ = dstBricks[i]->getPos3D().z;
				brickLayer = new BrickLayer();
			}
			brickLayer->addBrick(dstBricks[i]);
		}
		dstBrickLayerList.push_back(brickLayer);
		brickLayer = new BrickLayer();
	}
}

std::vector<BrickLayer*> JsonParser::getSrcBrickLayerList()
{
	return srcBrickLayerList;
}
std::vector<BrickLayer*> JsonParser::getDstBrickLayerList()
{
	return dstBrickLayerList;
}