/**
 * Brick.cpp
 * Purpose: Class encapsulates 'Brick'
 * @author Mintae Kim
 */

#include "Brick.h"

Brick::Brick()
{
	phase = BrickPhase::ENABLE;
	unableTerritoryCnt = 0;
}

void Brick::setAsEnable()
{
	unableTerritoryCnt--;
	if (unableTerritoryCnt == 0)
		phase = BrickPhase::ENABLE;
}
void Brick::setAsUnable()
{
	unableTerritoryCnt++;
	phase = BrickPhase::UNABLE;
}
void Brick::setAsSelected()
{
	phase = BrickPhase::SELECTED;
}
void Brick::setAsGrabbed()
{
	phase = BrickPhase::GRABBED;
}
void Brick::setAsLifted()
{
	phase = BrickPhase::LIFTED;
}
void Brick::setAsReleased()
{
	phase = BrickPhase::RELEASED;
}
void Brick::setAsDone()
{
	phase = BrickPhase::DONE;
}

void Brick::setPos(float x, float y, float z)
{
	pose.first.setVect3D(x, y, z);
}
void Brick::setDir(float x, float y, float z)
{
	pose.second.setVect3D(x, y, z);
}

Vector2D Brick::getPos2D()
{
	return pose.first.getVect2D();
}
Vector3D Brick::getPos3D()
{
	return pose.first;
}
Vector2D Brick::getDir2D()
{
	return pose.second.getVect2D();
}
Vector3D Brick::getDir3D()
{
	return pose.second;
}


BrickPhase Brick::getPhase()
{
	return phase;
}

bool Brick::operator <(const Brick& b2)
{
	return (pose.first.lowerPriorityThan(b2.pose.first));
}
bool Brick::operator ==(const Brick& b2)
{
	return pose.first == b2.pose.first;
}
bool Brick::operator > (const Brick& b2)
{
	return (!(*this == b2) && !(*this < b2));
}

std::string Brick::toString()
{
	return pose.first.toString();
}