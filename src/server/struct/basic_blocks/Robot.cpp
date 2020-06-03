/**
 * Robot.cpp
 * Purpose: Class encapsulates 'Robot' Information
 * @author Mintae Kim
 */

#include "Robot.h"

Robot::Robot(int robotNum, std::pair<Position2D, Direction2D>& initPose)
{
	this->robotNum = robotNum;
	this->pose = initPose;

	phase = RobotPhase::STOP;
	src = dst = NULL;
}

int Robot::getRobotNum()
{
	return robotNum;
}

void Robot::setPose(const std::pair<Position2D, Direction2D>& pose)
{
	this->pose = pose;
}
std::pair<Position2D, Direction2D>& Robot::getPose()
{
	return pose;
}

void Robot::setPath(const std::vector<std::pair<Position2D, Direction2D>>& path)
{
	this->path = path;
}
std::vector<std::pair<Position2D, Direction2D>>& Robot::getPath()
{
	return path;
}

std::pair<Position2D, Direction2D>& Robot::getKeypoint()
{
	if (path.empty()) {
		fprintf(stderr, "No Path Exists\n");
		return std::make_pair(Position2D(), Direction2D());
	}
	return path.front();
}
std::pair<Position2D, Direction2D>& Robot::getFinalPose()
{
	return finalPose;
}

Brick* Robot::getSourceBrick()
{
	return src;
}
Brick* Robot::getDestinationBrick()
{
	return dst;
}

void Robot::markAsMove(Brick* srcBrick, const std::pair<Position2D, Direction2D> &finalPose)
{
	phase = RobotPhase::MOVING;
	src = srcBrick;
	this->finalPose = finalPose;
}
void Robot::markAsGrab()
{
	phase = RobotPhase::GRAB;
}
void Robot::markAsLift(Brick* dstBrick, const std::pair<Position2D, Direction2D> &finalPose)
{
	phase = RobotPhase::LIFTING;
	dst = dstBrick;
	this->finalPose = finalPose;
}
void Robot::markAsRelease()
{
	phase = RobotPhase::RELEASE;
}
void Robot::markAsStop()
{
	phase = RobotPhase::STOP;
	src = dst = NULL;
}

std::string Robot::toString()
{
	std::string str;

	switch (phase)
	{
	case RobotPhase::STOP:
		str = "Stopped";
		break;
	case RobotPhase::MOVING:
		str = "Moving";
		break;
	case RobotPhase::GRAB:
		str = "Grabbing";
		break;
	case RobotPhase::LIFTING:
		str = "Lifting";
		break;
	case RobotPhase::RELEASE:
		str = "Releasing";
		break;
	}

	str.append(" ").append("Pos: ").append(pose.first.toString()).append(", Dir: ").append(pose.second.toString());

	return str;
}