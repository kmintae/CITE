/**
 * Direction.cpp
 * Purpose: Class encapsulates 'Direction Information'
 * @author Mintae Kim
 */

#include "Direction.h"

// 2D
void Direction2D::setDir2D(float x, float y)
{
	Vector2D::setVect2D(x, y);
}

const Direction2D Direction2D::getUnitVector()
{
	Vector2D unitVector = unitVector / (this->size());
	return (Direction2D)unitVector;
}

// Operator Overloading
const Direction2D Direction2D::operator +(const Direction2D& dir2)
{
	float radian1 = Direction2D::vectorToRadian(*this);
	float radian2 = Direction2D::vectorToRadian(dir2);

	return Direction2D::radianToVector(radian1 + radian2);
}
Direction2D& Direction2D::operator +=(const Direction2D& dir2)
{
	float radian1 = Direction2D::vectorToRadian(*this);
	float radian2 = Direction2D::vectorToRadian(dir2);

	Direction2D sum = Direction2D::radianToVector(radian1 + radian2);
	x = sum.x;
	y = sum.y;
	
	return *this;
}
const Direction2D Direction2D::operator -(const Direction2D& dir2)
{
	float radian1 = Direction2D::vectorToRadian(*this);
	float radian2 = Direction2D::vectorToRadian(dir2);

	return Direction2D::radianToVector(radian1 - radian2);
}
Direction2D& Direction2D::operator -=(const Direction2D& dir2)
{
	float radian1 = Direction2D::vectorToRadian(*this);
	float radian2 = Direction2D::vectorToRadian(dir2);

	Direction2D sub = Direction2D::radianToVector(radian1 - radian2);
	x = sub.x;
	y = sub.y;

	return *this;
}
const Direction2D Direction2D::operator +(const float radian2)
{
	float radian1 = Direction2D::vectorToRadian(*this);
	return Direction2D::radianToVector(radian1 + radian2);
}
Direction2D& Direction2D::operator +=(const float radian2)
{
	float radian1 = Direction2D::vectorToRadian(*this);

	Direction2D sum = Direction2D::radianToVector(radian1 + radian2);
	x = sum.x;
	y = sum.y;

	return *this;
}
const Direction2D Direction2D::operator -(const float radian2)
{
	float radian1 = Direction2D::vectorToRadian(*this);

	return Direction2D::radianToVector(radian1 - radian2);
}
Direction2D& Direction2D::operator -=(const float radian2)
{
	float radian1 = Direction2D::vectorToRadian(*this);

	Direction2D sub = Direction2D::radianToVector(radian1 - radian2);
	x = sub.x;
	y = sub.y;

	return *this;
}
bool Direction2D::operator ==(const Direction2D& dir2)
{
	return (((int)(abs(Direction2D::vectorToRadian(*this - dir2))) % 360) < errorLimit);
}
const Direction2D Direction2D::radianToVector(const float radian)
{
	return Direction2D(cos(radian), sin(radian));
}
const float Direction2D::vectorToRadian(const Direction2D& unitVector)
{
	return atan2(unitVector.y, unitVector.x);
}

// Vector
const float Direction2D::operator* (const Direction2D& dir2)
{
	return x * dir2.x + y * dir2.y;
}


// 3D
void Direction3D::setDir3D(float x, float y, float z)
{
	Vector3D::setVect3D(x, y, z);
}

Direction2D Direction3D::getDir2D()
{
	return Vector3D::getVect2D();
}

const Direction3D Direction3D::getUnitVector()
{
	return *this / (this->size());
}

// Operator Overloading
// Vector
const float Direction3D::operator* (const Direction3D& dir2)
{
	return x * dir2.x + y * dir2.y + z * dir2.z;
}