/**
 * Brick.h
 * Purpose: Class encapsulates 'Brick'
 * @author Mintae Kim
 */

#pragma once

#include <string>
#include <utility>

#include "Phase.h"

#include "../vector/Position.h"
#include "../vector/Direction.h"

class Brick
{
private:
    std::pair<Position3D, Direction3D> pose;

    BrickPhase phase; // Brick Phase

    int unableTerritoryCnt;

public:

    Brick();

    BrickPhase getPhase();
    void setAsEnable();
    void setAsUnable();
    void setAsSelected();
    void setAsGrabbed();
    void setAsLifted();
    void setAsReleased();
    void setAsDone();

    void setPos(float x, float y, float z);
    void setDir(float x, float y, float z);

    Position2D& getPos2D();
    Position3D& getPos3D();
    Direction2D& getDir2D();
    Direction3D& getDir3D();

    bool operator <(const Brick& b2);
    bool operator >(const Brick& b2);
    bool operator ==(const Brick& b2);
    std::string toString();
};