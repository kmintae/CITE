/**
 * BrickLayer.h
 * Purpose: Class encapsulates 'BrickLayer', Collection of Bricks in 'Same Level'
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <string>
#include <vector>
#include <Windows.h>

#include "Phase.h"
#include "Brick.h"

#include "../vector/Vector.h"

class BrickLayer
{
private:
    std::vector<Brick*> brickList;
    std::vector<Brick*> readyList, ongoingList, stackedList;

public:
    ~BrickLayer();

    void addBrick(Brick *nb);
    std::vector<Brick*> &getBrickList();
    std::vector<Brick*> getEnableBrickList();
    std::vector<Brick*> &getStackedList();

    void markAsSelected(Brick* b);
    void markAsDone(Brick* b);

    bool isDone();

    std::string toString();
};