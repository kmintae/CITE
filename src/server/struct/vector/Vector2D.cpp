/**
 * Vector2D.cpp
 * Purpose: Class encapsulates 'Vector 2D Information (x, y)'
 * @author Mintae Kim
 */

#include "Vector.h"

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

void Vector2D::callibrate()
{
	if (std::abs(x) < 0.001) x = 0;
	if (std::abs(y) < 0.001) y = 0;
}

float Vector2D::size()
{
	return x * x + y * y;
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

Vector2D& Vector2D::operator =(const Vector2D& vect2)
{
	x = vect2.x;
	y = vect2.y;
	return *this;
}

bool Vector2D::operator ==(const Vector2D& vect2)
{
	return ((x == vect2.x) && (y == vect2.y));
}
bool Vector2D::operator !=(const Vector2D& vect2)
{
	return !(*this == vect2);
}