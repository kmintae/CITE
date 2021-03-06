/**
 * Vector3D.cpp
 * Purpose: Class encapsulates 'Vector 3D Information (x, y, z)'
 * @author Mintae Kim
 */

#include "Vector.h"

Vector3D::Vector3D()
{
	this->z = 0;
}
Vector3D::Vector3D(float x, float y, float z) : Vector2D(x, y)
{
	this->z = z;
}
Vector3D::Vector3D(Vector3D const& vect) : Vector2D(vect)
{
	this->z = vect.z;
}

Vector2D Vector3D::getVect2D()
{
	return Vector2D(x, y);
}
void Vector3D::setVect3D(float x, float y, float z)
{
	Vector2D::setVect2D(x, y);
	this->z = z;
}

std::string Vector3D::toString()
{
	std::string str = "(";
	str.append(std::to_string(x)).append(", ").append(std::to_string(y)).append(", ").append(std::to_string(z));
	str.append(")");
	return str;
}
float Vector3D::size()
{
	return x * x + y * y + z * z;
}

void Vector3D::callibrate()
{
	Vector2D::callibrate();
	if (std::abs(z) < 0.001) z = 0;
}

const Vector3D Vector3D::unitVector()
{
	return *this / (this->size());
}
const float Vector3D::dotProduct(const Vector3D& vect1, const Vector3D& vect2)
{
	return vect1.x * vect2.x + vect1.y * vect2.y + vect1.z + vect2.z;
}
const float Vector3D::calculateDistance(const Vector3D& vect1, const Vector3D& vect2)
{
	float res = pow((vect1.x - vect2.x), 2.0) + pow((vect1.y - vect2.y), 2.0) + pow((vect1.z - vect2.z), 2.0);
	res = pow(res, 0.5);
	return res;
}

// Operator Overloading
const Vector3D Vector3D::operator +(const Vector3D& vect2)
{
	return Vector3D((x + vect2.x), (y + vect2.y), (z + vect2.z));
}
Vector3D& Vector3D::operator +=(const Vector3D& vect2)
{
	x += vect2.x;
	y += vect2.y;
	z += vect2.z;
	return *this;
}
const Vector3D Vector3D::operator -(const Vector3D& vect2)
{
	return Vector3D((x - vect2.x), (y - vect2.y), (z - vect2.z));
}
Vector3D& Vector3D::operator -=(const Vector3D& vect2)
{
	x -= vect2.x;
	y -= vect2.y;
	z -= vect2.z;
	return *this;
}

const Vector3D Vector3D::operator* (float mult)
{
	return Vector3D(x * mult, y * mult, z * mult);
}
Vector3D& Vector3D::operator *= (float mult)
{
	x *= mult;
	y *= mult;
	z *= mult;
	return *this;
}
const Vector3D Vector3D::operator/ (float div)
{
	return Vector3D(x / div, y / div, z / div);
}
Vector3D& Vector3D::operator /= (float div)
{
	x /= div;
	y /= div;
	z /= div;
	return *this;
}

bool Vector3D::operator==(const Vector3D& vect2)
{
	return (x == vect2.x && y == vect2.y && z == vect2.z);
}
bool Vector3D::operator!=(const Vector3D& vect2)
{
	return !(*this == vect2);
}

bool Vector3D::lowerPriorityThan(const Vector3D& vect2)
{
	if (z < vect2.z) return true;
	if (z > vect2.z) return false;

	if (y < vect2.y) return true;
	if (y > vect2.y) return false;

	if (x < vect2.x) return true;
	if (x > vect2.x) return false;

	return false;
}