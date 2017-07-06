//
//  RigidBody.hpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/1/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#ifndef RigidBody_hpp
#define RigidBody_hpp

#include <stdio.h>
#include <vector>
#include "Utils.h"

class RigidBody
{
public:
	RigidBody(Color c, Vector2d position, double halfwidth);
	RigidBody(float r, float g, float b, float alpha,
		int x, int y, double h);

	/** Time step to apply force and clear force accumulators */
	void advanceTime(double dt);

	/** Applies contact force in world coordinates */
	void applyContactForceW(Vector2d contactPoint, Vector2d contactForce);

	/** Accumulate force and torque in world space to affect next time step */
	void applyWrenchW(Vector2d f, double tau);

	/** Reset to initial state */
	void reset();

	/** Draws rigid body geometry */
	void display() const;

	/** Set whether this block has a pin constraint */
	void setPinned(bool isPin);

	// GETTERS
	double getH() const;
	double getRadius() const;
	double getMass() const;
	double getMassAngular() const;
	double getInvMass() const;
	double getInvMassAngular() const;
	bool isPinned() const;
	/* fragile! */
	Vector2d getPosition() const;
	double getOrientation() const;
	Vector2d getVelocity() const;
	double getVelocityAngular() const;
	Vector2d getForce() const;
	double getTorque() const;

	// fragile references to separating axes
	Vector2d getAxis1() const;
	Vector2d getAxis2() const;

	// calculate our vertices for this body and return
	std::vector<Vector2d> getVertices() const;

	Vector2d getSpatialVelocityW(Vector2d contactPointW);

	/** Rotate a vector by our current rotation matrix */
	Vector2d rotateByOrientation(Vector2d v) const;

	/** CONSTRAINT SOLVER UPDATE VARIABLES */
	// Linear change in velocity due to constraint impulses
	Vector2d deltaLinV;

	// Angular change in velocity due to constraint impulses
	double deltaAngV = 0;

	// collision processor index
	int colIndex;

private:
	/** Applies body to world transformation. */
	void applyGLTransform() const;

	/** Update rotation matrix. */
	void updateRotMat();

	/** Update separating axes. Note: to be performed after rotation matrix update. */
	void updateSeparatingAxes();

	/** Constant quantities */
	// Standard mass
	double massLinear = 1;

	// Angular mass
	double massAngular = 1;

	// Half width. radius is sqrt(2)*h
	double h;

	// Color of block
	Color color;

	// pinned?
	bool pinned = false;

	/** State variables */
	// Center of mass
	Vector2d x;

	// Initial position of center of mass
	Vector2d x0;

	/** Derived quantities */
	// Linear velocity in world frame
	Vector2d v;

	// Angular velocity
	double omega = 0;

	// Orientation angle
	double theta = 0;

	// Torque (angular force) accumulation in world frame
	double torque = 0;

	// Linear force accumulation in world frame
	Vector2d force;

	// Rotation matrix
	Vector2d rX = Vector2d(1,0);
	Vector2d rY = Vector2d(0,1);

	// Separating axes for collision
	Vector2d axis1;
	Vector2d axis2;
	
};

#endif /* RigidBody_hpp */
