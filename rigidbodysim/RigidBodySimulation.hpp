//
//  RigidBodySimulation.hpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/3/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#ifndef RigidBodySimulation_hpp
#define RigidBodySimulation_hpp

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <OpenGL/glu.h>
#include <vector>
#include <stdio.h>

#include "RigidBodySystem.hpp"

class RigidBodySimulation
{
public:
	/** Basic constructor */
	RigidBodySimulation();

	/** Destructor, frees rigid body system */
	~RigidBodySimulation();

	/**
	 * Creates an initial scene with walls.
	 */
	void setScene();

	/**
     * Builds and shows window, and starts simulator.
     */
	void start();

	/**
	 * Deletes rigid body system and starts over.
	 */
	void reset();
	
private:
	/** 
     * OpenGL display and simulation advance.
     */
	void display();

	/** Initializes SDL application, returns true on success. */
	bool init();

	/** Initializes OpenGL context. */
	bool initGL();

	/** Destroys and frees SDL window. */
	void close();

	/** Simulate then display rigid body system. */
	void simulateAndDisplaySystem();

	/** Take a time step. */
	void advanceTime(double dt);

	/** Create pinned blocks for floor and walls. */
	void createWalls();

	/** Spawn a falling block from the ceiling. */
	void addFallingBlock();

	/** Helper: get a random color (with alpha 1). */
	Color randomColor() const;

	/** Helper: get a random value in range (using rand()) */
	double randomInRange(double min, double max) const;

	/** Current simulation time. */
	double simTime = 0;
	/** last time a block fell from the sky */
	double lastBlockTime = 0;
	/** are we simulating/advancing time? */
	bool simulate = true;
	/** need to reset simulation? */
	bool needsReset = false;

	/** RigidBodySystem (singleton) reference */
	RigidBodySystem* rbs = nullptr;

	/** RENDERING AND WINDOWING */
	// The window we'll be rendering to
	SDL_Window* window = nullptr;

	// OpenGL context
	SDL_GLContext glContext;
};

#endif /* RigidBodySimulation_hpp */
