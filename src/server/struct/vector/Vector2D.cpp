/**
 * Vector2D.cpp
 * Purpose: Class encapsulates 'Vector 2D Information (x, y)'
 * @author Mintae Kim
 */

#include "Vector.h"

float Vector2D::dirLimit = (float)(GetPrivateProfileInt("error", "DIR_LIMIT_DEG", -1, "../config/server.ini")) / (180.0 / 3.141592);
float Vector2D::posNearestLimit = (float)GetPrivateProfileInt("error", "POS_LIMIT_MM", -1, "../config/server.ini");
float Vector2D::posNearLimit = (float)GetPrivateProfileInt("grid", "GRID_LEN_MM", -1, "../config/server.ini");

Vector2D::Vector2D()
{
	this->x = 0;
	this->y = 0;
}
Vector2D::Vector2D(float x, float y)
{
	this->x = x;
	this->y = y;
}
Vector2D::Vector2D(Vector2D const& vect)
{
	this->x = vect.x;
	this->y = vect.y;
}


void Vector2D::setVect2D(float x, float y)
{
	this->x = x;
	this->y = y;
}

std::string Vector2D::toString()
{
	std::string str = "(";
	str.append(std::to_string(x)).append(", ").append(std::to_string(y));
	str.append(")");
	return str;
}

float Vector2D::size()
{
	return x * x + y * y;
}

void Vector2D::callibrate()
{
	if (std::abs(x) < 0.001) x = 0;
	if (std::abs(y) < 0.001) y = 0;
}

const Vector2D Vector2D::unitVector()
{
	return *this / (this->size());
}
const float Vector2D::dotProduct(const Vector2D& vect1, const Vector2D& vect2)
{
	return vect1.x * vect2.x + vect1.y * vect2.y;
}
const float Vector2D::calculateDistance(const Vector2D& vect1, const Vector2D& vect2)
{
	float res = pow(pow((vect1.x - vect2.x), 2.0) + pow((vect1.y - vect2.y), 2.0), 0.5);
	return res;
}

const Vector2D Vector2D::radianToVector(const float radian)
{
	return Vector2D(cos(radian), sin(radian));
}
const float Vector2D::vectorToRadian(const Vector2D& uV)
{
	return atan2(uV.y, uV.x);
}

bool Vector2D::isSimilar(const Vector2D& vect1, const Vector2D& vect2) // Direction Degree Comparison
{
	float theta1 = Vector2D::vectorToRadian(vect1);
	float theta2 = Vector2D::vectorToRadian(vect2);

	float theta = (theta1 - theta2);
	float diff = abs(theta);
	if (diff > 180.0) {
		diff = 360 - diff;
	}
	return (diff < Vector2D::dirLimit);
}
bool Vector2D::isNearest(const Vector2D& vect1, const Vector2D& vect2) // Position Distance Comparison
{
	return Vector2D::calculateDistance(vect1, vect2) < Vector2D::posNearestLimit;
}
bool Vector2D::isNear(const Vector2D& vect1, const Vector2D& vect2) // Position Distance Comparison
{
	return Vector2D::calculateDistance(vect1, vect2) < Vector2D::posNearLimit;
}

// Operator Overloading
const Vector2D Vector2D::operator +(const Vector2D& vect2)
{
	return Vector2D((x + vect2.x), (y + vect2.y));
}
Vector2D& Vector2D::operator +=(const Vector2D& vect2)
{
	x += vect2.x;
	y += vect2.y;
	return *this;
}
const Vector2D Vector2D::operator -(const Vector2D& vect2)
{
	return Vector2D((x - vect2.x), (y - vect2.y));
}
Vector2D& Vector2D::operator -=(const Vector2D& vect2)
{
	x -= vect2.x;
	y -= vect2.y;
	return *this;
}

const Vector2D Vector2D::operator* (float mult)
{
	return Vector2D(x * mult, y * mult);
}
Vector2D& Vector2D::operator *= (float mult)
{
	x *= mult;
	y *= mult;
	return *this;
}
const Vector2D Vector2D::operator/ (float div)
{
	return Vector2D(x / div, y / div);
}
Vector2D& Vector2D::operator /= (float div)
{
	x /= div;
	y /= div;
	return *this;
}

bool Vector2D::operator==(const Vector2D& vect2)
{
	return (x == vect2.x && y == vect2.y);
}
bool Vector2D::operator!=(const Vector2D& vect2)
{
	return !(*this == vect2);
}