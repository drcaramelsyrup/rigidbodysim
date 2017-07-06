//
//  Utils.h
//  rigidbodysim
//
//  Created by Edward Wang on 12/1/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#ifndef Utils_h
#define Utils_h

#define _USE_MATH_DEFINES

#include <cmath>

namespace Constants
{
	/** numerical constants */
	/** epsilon for near zero diagonal values */
	static const double DISTANCE_EPSILON = 1.e-10;

	static const double SQRT_TWO = 1.414;
	static const double FRICTION_COEFFICIENT = 0.8;

	/** Size of symplectic Euler time step (in seconds) */
	static const double DT = 0.0001;

	/** Time steps per large step size */
	static const int N_STEPS_PER_FRAME = 15;
	// Screen dimension constants
	static const int SCREEN_WIDTH = 720;
	static const int SCREEN_HEIGHT = 720;

	/** half width of wall blocks. */
	static const double WALL_HALFWIDTH = 0.1;
	static const double LEFT_EDGE = -1.0;	// x
	static const double RIGHT_EDGE = 1.0;	// x
	static const double BOTTOM_EDGE = -1.0;	// y
	static const double TOP_EDGE = 1.0;		// y
	static const double FALLING_BLOCK_HALFWIDTH = 0.07;
	static const double FALLING_BLOCK_TORQUE_IMPULSE = 50000.0;
	static const double FALLING_BLOCK_TIME_INTERVAL = 0.3;

	/** PGS iteration count */
	static const int SOLVER_ITERATIONS = 60;
	/** PGS convergence epsilon */
	static const double CONVERGE_EPSILON = 1.e-10;
}

class Vector2d
{
public:
	// default constructors
	Vector2d()
		: x(0.0)
		, y(0.0)
	{
	}

	Vector2d(double xCoord, double yCoord)
		: x(xCoord)
		, y(yCoord)
	{
	}

	/** returns dot product value */
	static double dot(Vector2d first, Vector2d second)
	{
		return first.x*second.x + first.y*second.y;
	}

	/** returns cross product of two 2d vectors */
	static double cross(Vector2d first, Vector2d second)
	{
		return first.x*second.y - first.y*second.x;
	}

	/** returns projection of a onto b */
	static Vector2d projection(Vector2d a, Vector2d b)
	{
		double dp = dot(a, b) / b.lengthSq();
		return Vector2d(dp*b.x, dp*b.y);
	}

	/** returns length of current vector */
	inline double length() { return std::sqrt(x*x + y*y); }

	/** returns length squared of current vector */
	inline double lengthSq() { return x*x + y*y; }

	// overloaded operators
	Vector2d& operator+=(const Vector2d& v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}

	friend Vector2d operator+(Vector2d lhs, const Vector2d& rhs)
	{
		lhs += rhs;
		return lhs;
	}

	Vector2d& operator-=(const Vector2d& v)
	{
		x -= v.x;
		y -= v.y;
		return *this;
	}

	friend Vector2d operator-(Vector2d lhs, const Vector2d& rhs)
	{
		lhs -= rhs;
		return lhs;
	}

	void normalize()
	{
		double toUnit = 1.0f / length();
		x *= toUnit;
		y *= toUnit;
	}

	void projectOnto(Vector2d b)
	{
		double dp = (x*b.x + y*b.y) / b.lengthSq();
		x = dp*b.x;
		y = dp*b.y;
	}

	void negate() { x = -x; y = -y; }
	Vector2d negated() const { return Vector2d(-x, -y); }

	/** curr += v */
	void add(Vector2d v) { x += v.x; y += v.y; }

	/** curr -= v */
	void sub(Vector2d v) { x -= v.x; y -= v.y; }

	/** curr *= scale */
	void scale(double scale) { x *= scale; y *= scale; }

	/** curr += scale*v */
	void acc(double scale, Vector2d v)
	{
		x += scale*v.x;
		y += scale*v.y;
	}

	/** set curr to value of other vector or values */
	void set(Vector2d v) { x = v.x; y = v.y; }
	void set(double newX, double newY) { x = newX; y = newY; }
	void setZero() { x = 0.0; y = 0.0; }

	double x = 0.0;
	double y = 0.0;
};

struct Color
{
	float r = 0xFF;
	float g = 0xFF;
	float b = 0xFF;
	float alpha = 0xFF;

	// default constructors
	Color()
		: r(0xFF)
		, g(0xFF)
		, b(0xFF)
		, alpha(0xFF)
	{
	}

	Color(float red, float green, float blue, float alphaVal)
		: r(red)
		, g(green)
		, b(blue)
		, alpha(alphaVal)
	{
	}
};

#endif /* Utils_h */
