/**
 * Robot.cpp
 * Purpose: Class encapsulates 'Robot' Information
 * @author Mintae Kim
 */

#include "Robot.h"

Robot::Robot(int robotNum, std::pair<Vector2D, Vector2D>& initPose)
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

void Robot::setPose(const std::pair<Vector2D, Vector2D>& pose)
{
	this->pose = pose;
}
std::pair<Vector2D, Vector2D> Robot::getPose()
{
	return pose;
}

void Robot::setPath(const std::vector<std::pair<Vector2D, Vector2D>>& path)
{
	this->path = path;
}
std::vector<std::pair<Vector2D, Vector2D>> Robot::getPath()
{
	return path;
}

std::pair<Vector2D, Vector2D> Robot::getKeypoint()
{
	return path.front();
}
std::pair<Vector2D, Vector2D> Robot::getFinalPose()
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

void Robot::markAsMove(Brick* srcBrick, const std::pair<Vector2D, Vector2D> &finalPose)
{
	phase = RobotPhase::MOVING;
	src = srcBrick;
	this->finalPose = finalPose;
}
void Robot::markAsGrab()
{
	phase = RobotPhase::GRAB;
}
void Robot::markAsLift(Brick* dstBrick, const std::pair<Vector2D, Vector2D> &finalPose)
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