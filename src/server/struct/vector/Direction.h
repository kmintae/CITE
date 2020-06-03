/**
 * Direction.h
 * Purpose: Class encapsulates 'Direction Information (2D/3D)'
 * @author Mintae Kim
 */

#pragma once
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <cmath>
#include <Windows.h>

#include "Vector.h"

class Direction2D : public Vector2D
{
protected:
	float errorLimit;

public:
	Direction2D() {
		errorLimit = GetPrivateProfileInt("error", "DIR_LIMIT_DEG", 2, "../../../config/server.ini");
		errorLimit /= (180.0 / 3.141592);
	};
	Direction2D(float dir_x, float dir_y) : Vector2D(dir_x, dir_y) { 
		errorLimit = GetPrivateProfileInt("error", "DIR_LIMIT_DEG", 2, "../../../config/server.ini");
		errorLimit /= (180.0 / 3.141592);
	};
	Direction2D(Direction2D const& dir) : Vector2D(dir) { 
		errorLimit = GetPrivateProfileInt("error", "DIR_LIMIT_DEG", 2, "../../../config/server.ini");
		errorLimit /= (180.0 / 3.141592);
	};

	void setDir2D(float x, float y);

	const Direction2D getUnitVector();

	// Operator Overloading
	const Direction2D operator +(const Direction2D& dir2);
	Direction2D& operator +=(const Direction2D& dir2);
	const Direction2D operator -(const Direction2D& dir2);
	Direction2D& operator -=(const Direction2D& dir2);
	const Direction2D operator +(const float radian2);
	Direction2D& operator +=(const float radian2);
	const Direction2D operator -(const float radian2);
	Direction2D& operator -=(const float radian2);

	bool operator ==(const Direction2D& dir2);

	static const Direction2D radianToVector(const float radian);
	static const float vectorToRadian(const Direction2D &unitVector);

	// Additional Operator
	const float operator* (const Direction2D& dir2);
};

// Inheriting Vector3D Only
class Direction3D : public Vector3D
{
public:
	Direction3D() { };
	Direction3D(float dir_x, float dir_y, float dir_z) : Vector3D(dir_x, dir_y, dir_z) { };
	Direction3D(Direction3D const& dir) : Vector3D(dir) { };

	Direction2D getDir2D();

	void setDir3D(float x, float y, float z);

	const Direction3D getUnitVector();

	const float operator* (const Direction3D& dir2);
};