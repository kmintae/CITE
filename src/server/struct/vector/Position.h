/**
 * Position.h
 * Purpose: Class encapsulates 'Position Information (2D/3D)'
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <string>
#include <cmath>
#include <Windows.h>

#include "Vector.h"

class Position2D : public Vector2D
{
private:
	float errorLimit;
public:
	Position2D() {
		errorLimit = GetPrivateProfileInt("error", "POS_LIMIT_MM", 2, "../../../config/server.ini");
	};
	Position2D(float pos_x, float pos_y) : Vector2D(pos_x, pos_y) {
		errorLimit = GetPrivateProfileInt("error", "POS_LIMIT_MM", 2, "../../../config/server.ini");
	};
	Position2D(Position2D const& pos) : Vector2D(pos) {
		errorLimit = GetPrivateProfileInt("error", "POS_LIMIT_MM", 2, "../../../config/server.ini");
	};

	static float calculateDistance(const Position2D& pos1, const Position2D& pos2);

	void setPos2D(float x, float y);

	// Operator Overloading
	bool operator ==(const Position2D& pos2);
};

// Only Inherits Vector3D
class Position3D : public Vector3D
{
private:
	float errorLimit;
public:
	Position3D() { };
	Position3D(float pos_x, float pos_y, float pos_z) : Vector3D(pos_x, pos_y, pos_z){ };
	Position3D(Position3D const& pos) : Vector3D(pos){ };

	Position2D getPos2D();

	static float calculateDistance(const Position3D& pos1, const Position3D& pos2);

	void setPos3D(float x, float y, float z);

	// Additional Operator Overloading
	bool operator <(const Position3D& pos2);
	bool operator >(const Position3D& pos2);

	bool operator ==(const Position3D& pos2);
};