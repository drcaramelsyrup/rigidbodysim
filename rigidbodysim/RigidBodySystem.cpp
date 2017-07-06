//
//  RigidBodySystem.cpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/3/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#include "RigidBodySystem.hpp"

RigidBodySystem::RigidBodySystem()
{
}

RigidBodySystem::~RigidBodySystem()
{
	// free rigid bodies
	for (RigidBody* body : bodies)
	{
		if (body != nullptr)
			delete body;
		body = nullptr;
	}
	if (collisionProcessor != nullptr)
		delete collisionProcessor;
	collisionProcessor = nullptr;
}

void RigidBodySystem::add(RigidBody* rb) 
{
	if (rb == nullptr)
	{
		printf("Error: adding a null body\n");
		return;
	}

	bodies.insert(rb);
	// printf("Block count: %d\n", bodies.size());

	// CollisionProcessor needs to be rebuilt at timestep!
	// use this list to add new bodies
	newBodyQ.push(rb);
}

void RigidBodySystem::remove(RigidBody* rb) 
{
	if (rb == nullptr)
	{
		printf("Error: removing an invalid body\n");
		return;
	}

	bodies.erase(rb);
	// free memory associated with rigid body
	delete rb;
	rb = nullptr;

	/// CollisionProcessor needs to be rebuilt at timestep!
	// use this list to add new bodies
	// remove index from collision processor. not implemented
}

void RigidBodySystem::reset()
{
	for (RigidBody* rb : bodies)
	{
		rb->reset();
	}
	time = 0;
}

void RigidBodySystem::advanceTime(double dt)
{
	time += dt;
 	/// NOTE: Clear force accumulators: already done after time step in RigidBody

    // GRAVITY
    Vector2d f(0, 0);
	double   tau = 0;
	for (RigidBody* body : bodies)
	{
		f.x = f.y = 0;
		double m = body->getMass();

		f.y -= m * 10;//gravity

		body->applyWrenchW(f, tau);
	}

	/// RESOLVE COLLISIONS!
    if (collisionProcessor == nullptr)
    {
    	collisionProcessor = new CollisionProcessor();
    }
    // any new bodies to add?
    while (!newBodyQ.empty())
    {
    	RigidBody* newBody = newBodyQ.front();
    	collisionProcessor->addBody(newBody);
    	newBodyQ.pop();
    }
    collisionProcessor->processCollisions();

	for (RigidBody* body : bodies)
	{
		// velocity (and position) update.
		// clear force/delta constraint accumulators
		body->advanceTime(dt);
	}

	time += dt;
}

/**
 * Displays RigidBody and Force objects.
 */
void RigidBodySystem::display() const
{
	for (RigidBody* body : bodies)
	{
		if (body == nullptr)
		{
			printf("Error: rigid body in system is null!\n");
			continue;
		}

		body->display();
	}

}

/** Picks body based on some criteria, or null if none picked.  */
RigidBody* RigidBodySystem::pickBody(Vector2d p)
{
	// double    pickDist = Double.MAX_VALUE;
	// RigidBody pick     = null;
	// for(RigidBody body : bodies) {

	// 	if(body.isPinned()) continue;

	// 	double dist = body.getPosition().distance(p);
	// 	if(dist < pickDist) {
	// 		pickDist = dist;
	// 		pick     = body;
	// 	}
	// }
	// return pick;
	return nullptr;
}

std::unordered_set<RigidBody*> RigidBodySystem::getRigidBodies() const
{
	return bodies;
}

int RigidBodySystem::getNBodies() const
{
	return bodies.size();
}