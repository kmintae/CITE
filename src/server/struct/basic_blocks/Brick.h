/**
 * Brick.h
 * Purpose: Class encapsulates 'Brick'
 * @author Mintae Kim
 */

#pragma once

#include <string>
#include <utility>

#include "Phase.h"

#include "../vector/Vector.h"

class Brick
{
private:
    std::pair<Vector3D, Vector3D> pose;

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

    Vector2D& getPos2D();
    Vector3D& getPos3D();
    Vector2D& getDir2D();
    Vector3D& getDir3D();

    bool operator <(const Brick& b2);
    bool operator >(const Brick& b2);
    bool operator ==(const Brick& b2);
    std::string toString();
};