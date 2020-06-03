/**
 * Vector.h
 * Purpose: Class encapsulates 'Vector' Information
 * @author Mintae Kim
 */

#pragma once

#include <string>
#include <cmath>

class Vector2D
{
public:
	float x, y;
	
	Vector2D();
	Vector2D(float x, float y);
	Vector2D(Vector2D const& vect);

	void setVect2D(float x, float y);
	std::string toString();
	
	float size();
	void callibrate();

	const Vector2D operator +(const Vector2D& vect2);
	Vector2D& operator +=(const Vector2D& vect2);
	const Vector2D operator -(const Vector2D& vect2);
	Vector2D& operator -=(const Vector2D& vect2);

	const Vector2D operator* (float mult);
	Vector2D& operator *= (float mult);
	const Vector2D operator/ (float div);
	Vector2D& operator /= (float div);

	Vector2D& operator =(const Vector2D& vect2);
	bool operator ==(const Vector2D& vect2);
	bool operator !=(const Vector2D& vect2);
};

class Vector3D : public Vector2D
{
public:
	float z;

	Vector3D();
	Vector3D(float x, float y, float z);
	Vector3D(Vector3D const& vect);

	Vector2D getVect2D();

	void setVect3D(float x, float y, float z);
	static float calculateDistance(const Vector3D& vect1, const Vector3D& vect2);
	std::string toString();

	float size();
	void callibrate();

	const Vector3D operator +(const Vector3D& vect2);
	Vector3D& operator +=(const Vector3D& vect2);
	const Vector3D operator -(const Vector3D& vect2);
	Vector3D& operator -=(const Vector3D& vect2);

	const Vector3D operator* (float mult);
	Vector3D& operator *= (float mult);
	const Vector3D operator/ (float div);
	Vector3D& operator /= (float div);

	Vector3D& operator =(const Vector3D& vect2);
	bool operator ==(const Vector3D& vect2);
	bool operator !=(const Vector3D& vect2);
};