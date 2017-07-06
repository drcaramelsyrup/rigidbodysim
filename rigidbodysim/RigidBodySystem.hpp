//
//  RigidBodySystem.hpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/3/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#ifndef RigidBodySystem_hpp
#define RigidBodySystem_hpp

#include <stdio.h>
#include <queue>
#include <unordered_set>
#include "CollisionProcessor.hpp"
#include "RigidBody.hpp"

class RigidBodySystem
{
public:
	/** Basic constructor. */
	RigidBodySystem();

	/** Basic destructor. */
	~RigidBodySystem();

	/** Adds the RigidBody to the system, and invalidates the existing
 	* CollisionProcessor. */
	void add(RigidBody* rb);

	/** Removes the RigidBody from the system, and invalidates the
	 * existing CollisionProcessor. */
	void remove(RigidBody* rb);

	/** Moves all rigidbodys to undeformed/materials positions, and
	 * sets all velocities to zero. Synchronized to avoid problems
	 * with simultaneous calls to advanceTime(). */
	void reset();

	/**
	 * Integrator implementation based on
	 * the velocity-level complementarity constraint solver.
	 */
	void advanceTime(double dt);

	/**
	 * Displays RigidBody and Force objects.
	 */
	void display() const;

	/** Picks body based on some criteria, or null if none picked.  */
	RigidBody* pickBody(Vector2d point);

	/** Get reference to rigid bodies.  */
	std::unordered_set<RigidBody*> getRigidBodies() const;

	/** Number of rigid bodies. */
	int getNBodies() const;

private:
	/** Current simulation time. */
	double time = 0;

	/** Do we need to rebuild the collision processor? read: add a new body */
	std::queue<RigidBody*> newBodyQ;

	/** List of RigidBody objects (pointers). */
	std::unordered_set<RigidBody*> bodies;

	CollisionProcessor* collisionProcessor = nullptr;
};

#endif /* RigidBodySystem_hpp */
