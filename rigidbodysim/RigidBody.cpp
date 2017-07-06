//
//  RigidBody.cpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/1/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#include <cmath>
#include <OpenGL/glu.h>
#include "RigidBody.hpp"

RigidBody::RigidBody(Color c, Vector2d position, double halfwidth)
	: color(c)
	, x0(position)
	, h(halfwidth)
{
	reset();
}

RigidBody::RigidBody(float r, float g, float b, float alpha,
	int x, int y, double halfwidth)
{
	color.r = r;
	color.g = g;
	color.b = b;
	color.alpha = alpha;

	this->x0.x = x;
	this->x0.y = y;
	
	h = halfwidth;

	reset();
}

double RigidBody::getH() const { return h; }
double RigidBody::getRadius() const { return Constants::SQRT_TWO * h; }
// support infinite mass for pinned objects ()
double RigidBody::getMass() const { return massLinear; }
double RigidBody::getMassAngular() const { return massAngular; } 
double RigidBody::getInvMass() const { return pinned ? 0 : 1.0/massLinear; } 
double RigidBody::getInvMassAngular() const { return pinned ? 0 : 1.0/massAngular; } 
bool RigidBody::isPinned() const { return pinned; }
Vector2d RigidBody::getPosition() const { return x; } 
double RigidBody::getOrientation() const { return theta; } 
Vector2d RigidBody::getVelocity() const { return v; }
double RigidBody::getVelocityAngular() const { return omega; }
Vector2d RigidBody::getForce() const { return force; }
double RigidBody::getTorque() const { return torque; }
Vector2d RigidBody::getAxis1() const { return axis1; }
Vector2d RigidBody::getAxis2() const { return axis2; }

std::vector<Vector2d> RigidBody::getVertices() const
{
	// consider the square. what a nice shape
	std::vector<Vector2d> vertices;
	Vector2d v0(x);
	v0.add(axis1);
	v0.add(axis2);
	Vector2d v1(x);
	v1.add(axis1);
	v1.sub(axis2); 
	Vector2d v2(x);
	v2.sub(axis1);
	v2.sub(axis2);
	Vector2d v3(x);
	v3.sub(axis1);
	v3.add(axis2);

	vertices.push_back(v0);
	vertices.push_back(v1);
	vertices.push_back(v2);
	vertices.push_back(v3);

	return vertices;
}

Vector2d RigidBody::getSpatialVelocityW(Vector2d contactPointW)
{
	// rotational velocity:
	Vector2d pv(contactPointW);
	pv.sub(x);
	pv.scale(omega);
	double vy = pv.y;
	pv.y = pv.x;
	pv.x = -vy;
	// linear velocity:
	pv.add(v);

	return pv;
}

Vector2d RigidBody::rotateByOrientation(Vector2d v) const
{
	return Vector2d(Vector2d::dot(rX, v), Vector2d::dot(rY, v));
}

void RigidBody::advanceTime(double dt)
{
	if (pinned)
		return;
	// symplectic Euler
	// printf("[%d] advanceTime (pre):	v=(%f, %f), x=(%f, %f)\n", colIndex, v.x, v.y, x.x, x.y);

	v.add(deltaLinV);	// according to collision constraints
	v.acc(dt*getInvMass(), force);	// external forces
	x.acc(dt, v);

	// printf("[%d] advanceTime (post):	v=(%f, %f), x=(%f, %f)\n", colIndex, v.x, v.y, x.x, x.y);

	// update angular position and velocity
	omega += (deltaAngV);	// according to collision constraints
	omega += (dt*getInvMassAngular() * torque);	// external forces
	theta += (dt * omega);

	// reset accumulators
	force.x = force.y = torque = 0;
	deltaLinV.x = deltaLinV.y = deltaAngV = 0;

	updateRotMat();
	updateSeparatingAxes();
}

void RigidBody::updateRotMat()
{
	// update rotation matrix
	rX.x = std::cos(theta);
	rX.y = -std::sin(theta);
	rY.x = -rX.y;
	rY.y = rX.x;
}

void RigidBody::updateSeparatingAxes()
{
	axis1 = rotateByOrientation(Vector2d(h, 0));
	axis2 = rotateByOrientation(Vector2d(0, h));
}

void RigidBody::applyContactForceW(Vector2d contactPoint, Vector2d contactForce)
{
	force.add(contactForce);
	// add torque: zHat dot (x-p) cross ext_f
	Vector2d relPos(x);
	relPos.sub(contactPoint);
	torque += Vector2d::cross(relPos, force);
}

void RigidBody::applyWrenchW(Vector2d f, double tau)
{
	force.add(f);
	torque += tau;
}

void RigidBody::reset()
{
	x.set(x0);
	deltaLinV.set(0, 0);
	deltaAngV = 0;
	v.x = v.y = theta = omega = force.x = force.y = torque = 0;

	updateRotMat();
	updateSeparatingAxes();
}

void RigidBody::display() const
{
	glPushMatrix();
	applyGLTransform();

	// draw block
	glBegin(GL_QUADS);
		glColor3f(color.r, color.g, color.b);

		glVertex2d(h, h);
		glVertex2d(h, -h);
		glVertex2d(-h, -h);
		glVertex2d(-h, h);
	glEnd();

	glPopMatrix();

	// draw separating axes
	glPushMatrix();
	glTranslated(x.x, x.y, 0);
	glBegin(GL_LINES);
		glColor3f(1.0-color.r, 1.0-color.g, 1.0-color.b);
		glVertex2d(0, 0);
		
		glVertex2d(axis1.x, axis1.y);

		glVertex2d(0, 0);
		
		glVertex2d(axis2.x, axis2.y);
	glEnd();
	glPopMatrix();
}

void RigidBody::applyGLTransform() const
{
	glTranslated(x.x, x.y, 0);
	double angleInDegrees = 180./M_PI * theta;
	glRotated(angleInDegrees, 0, 0, 1);
}

void RigidBody::setPinned(bool isPin)
{
	pinned = isPin;
}
