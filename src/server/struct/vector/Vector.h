/**
 * Vector.h
 * Purpose: Class encapsulates 'Vector' Information
 * @author Mintae Kim
 */

#pragma once

#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include <string>
#include <cmath>
#include <Windows.h>

class Vector2D
{
protected:
	static float dirLimit, posNearestLimit, posNearLimit;

public:
	float x, y;
	
	Vector2D();
	Vector2D(float x, float y);
	Vector2D(Vector2D const& vect);

	void setVect2D(float x, float y);
	std::string toString();
	
	float size();
	void callibrate();

	const Vector2D unitVector();
	const static float dotProduct(const Vector2D& vect1, const Vector2D& vect2);
	const static float calculateDistance(const Vector2D& vect1, const Vector2D& vect2);

	const static Vector2D radianToVector(const float radian);
	const static float vectorToRadian(const Vector2D& uV);

	static bool isSimilar(const Vector2D& vect1, const Vector2D& vect2); // Direction Degree Comparison
	static bool isNearest(const Vector2D& vect1, const Vector2D& vect2); // Position Distance Comparison
	static bool isNear(const Vector2D& vect1, const Vector2D& vect2); // Position Distance Comparison

	const Vector2D operator +(const Vector2D& vect2);
	Vector2D& operator +=(const Vector2D& vect2);
	const Vector2D operator -(const Vector2D& vect2);
	Vector2D& operator -=(const Vector2D& vect2);

	const Vector2D operator* (float mult);
	Vector2D& operator *= (float mult);
	const Vector2D operator/ (float div);
	Vector2D& operator /= (float div);

	bool operator==(const Vector2D& vect2);
	bool operator!=(const Vector2D& vect2);
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
	
	std::string toString();

	float size();
	void callibrate();

	const Vector3D unitVector();
	const static float dotProduct(const Vector3D& vect1, const Vector3D& vect2);
	const static float calculateDistance(const Vector3D& vect1, const Vector3D& vect2);

	const Vector3D operator +(const Vector3D& vect2);
	Vector3D& operator +=(const Vector3D& vect2);
	const Vector3D operator -(const Vector3D& vect2);
	Vector3D& operator -=(const Vector3D& vect2);

	const Vector3D operator* (float mult);
	Vector3D& operator *= (float mult);
	const Vector3D operator/ (float div);
	Vector3D& operator /= (float div);

	bool operator==(const Vector3D& vect2);
	bool operator!=(const Vector3D& vect2);

	bool lowerPriorityThan(const Vector3D& vect2);
};