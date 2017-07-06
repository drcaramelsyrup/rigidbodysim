//
//  RigidBodySimulation.cpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/3/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#include "RigidBodySimulation.hpp"

RigidBodySimulation::RigidBodySimulation()
{
	rbs = new RigidBodySystem();
}

RigidBodySimulation::~RigidBodySimulation()
{
	if (rbs != nullptr)
		delete rbs;
	rbs = nullptr;
}

void RigidBodySimulation::start()
{
	setScene();
	// Initialize SDL
	init();
	display();
}

void RigidBodySimulation::reset()
{
	if (rbs != nullptr)
	{
		RigidBodySystem* garbage = rbs;
		rbs = nullptr;
		delete garbage;	
	}

	rbs = new RigidBodySystem();
	setScene();
	simTime = 0;
	lastBlockTime = 0;
	simulate = false;
}

void RigidBodySimulation::display()
{
	// main loop flag
	bool quit = false;

	// event handler
	SDL_Event e;

	// main loop
	while (!quit)
	{
		// handle events on queue
		while (SDL_PollEvent(&e) != 0)
		{
			if (e.type == SDL_QUIT)
			{
				quit = true;
			}
			// key presses
			else if (e.type == SDL_KEYDOWN)
			{
				switch( e.key.keysym.sym )
	            {
	                case SDLK_SPACE:
						simulate = !simulate;
						break;
					case SDLK_ESCAPE:
						needsReset = !needsReset;
						break;
				}
			}
		}

		glDisable(GL_DEPTH_TEST);
		glClear(GL_COLOR_BUFFER_BIT); //  | GL.GL_DEPTH_BUFFER_BIT);

		/// DRAW COMPUTATIONAL CELL BOUNDARY:
	    glBegin(GL_LINE_STRIP);
			glColor4f(0,0,0,1);
	    	glVertex2d(0,0);
	    	glVertex2d(1,0);
	    	glVertex2d(1,1);
	    	glVertex2d(0,1);
	    	glVertex2d(0,0);
	    glEnd();
	            
		glPolygonMode(GL_FRONT, GL_FILL);

		// set multisampling
		glEnable(GL_BLEND);
		glEnable(GL_MULTISAMPLE);
		
		// Render system
		simulateAndDisplaySystem();

		glDisable(GL_MULTISAMPLE);

		// Update screen
		SDL_GL_SwapWindow(window);

	}

	// cleanup.
	close();
}

/**
 * INITIALIZATION, INPUT, and WINDOWING 
 */
bool RigidBodySimulation::init()
{
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		return false;
	}

	// OpenGL multisampling settings
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);

	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	// Create window
	window = SDL_CreateWindow("rigidbodysim", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
		Constants::SCREEN_WIDTH, Constants::SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
	if (window == nullptr)
	{
		printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		return false;
	}

	// Create OpenGL context
	glContext = SDL_GL_CreateContext(window);
	if (glContext == nullptr)
	{
		printf("OpenGL context could not be created! SDL_Error: %s\n", SDL_GetError());
		return false;
	}
	// Use Vsync
	if (SDL_GL_SetSwapInterval(1) < 0)
	{
		printf("Warning: Unable to set VSync! SDL Error: %s\n", SDL_GetError());
	}

	// Initialize OpenGL
	if (!initGL())
	{
		printf("Unable to initialize OpenGL!\n");
		return false;
	}

	return true;
}

bool RigidBodySimulation::initGL()
{
	bool success = true;
	GLenum error = GL_NO_ERROR;

	// Initialize projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Check for errors
	error = glGetError();
	if (error != GL_NO_ERROR)
	{
		printf("Error initializing OpenGL! %s\n", gluErrorString(error));
		success = false;
	}

	// Initialize Modelview Matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	error = glGetError();
	if (error != GL_NO_ERROR)
	{
		printf("Error initializing OpenGL! %s\n", gluErrorString(error));
		success = false;
	}
	
	// Initialize clear color
	glClearColor(1.f, 1.f, 1.f, 0.f);
	
	error = glGetError();
	if (error != GL_NO_ERROR)
	{
		printf("Error initializing OpenGL! %s\n", gluErrorString(error));
		success = false;
	}
	
	return success;
}

void RigidBodySimulation::close()
{
	// Destroy window
	SDL_DestroyWindow( window );
	window = nullptr;

	// Quit SDL subsystems
	SDL_Quit();
}

/**
 * SIMULATION 
 */
void RigidBodySimulation::simulateAndDisplaySystem()
{
	if (needsReset)
	{
		reset();
		needsReset = false;
	}
	if (simulate)
	{
		// one Euler step
		for (int k = 0; k < Constants::N_STEPS_PER_FRAME; ++k)
		{
			advanceTime(Constants::DT);		
		}
		// every interval, add a falling block
		if (simTime - lastBlockTime > Constants::FALLING_BLOCK_TIME_INTERVAL)
		{
			addFallingBlock();
			lastBlockTime = simTime;
		}
	}	

	// clear color buffer
	glClear(GL_COLOR_BUFFER_BIT);

	if (rbs == nullptr)
	{
		printf("Error: rigid body system is null!\n");
		return;
	}

	// render quad
	rbs->display();
}

void RigidBodySimulation::advanceTime(double dt)
{
	if (rbs == nullptr)
	{
		printf("Error: rigid body system is null!\n");
		return;
	}

	rbs->advanceTime(dt);
	simTime += dt;
}

void RigidBodySimulation::setScene()
{
	if (rbs == nullptr)
	{
		printf("Error: rigid body system is null when setting scene!\n");
		return;
	}

	createWalls();
}

void RigidBodySimulation::addFallingBlock()
{
	// add random torque impulse
	double minTorque = -Constants::FALLING_BLOCK_TORQUE_IMPULSE;
	double maxTorque = Constants::FALLING_BLOCK_TORQUE_IMPULSE;
	double tau = randomInRange(minTorque, maxTorque);

	// halfwidth with random deviation of 20%
	double widthDeviation = 0.2*Constants::FALLING_BLOCK_HALFWIDTH;
	double halfwidth = randomInRange(Constants::FALLING_BLOCK_HALFWIDTH - widthDeviation,
		Constants::FALLING_BLOCK_HALFWIDTH + widthDeviation);

	double minX = Constants::LEFT_EDGE + 2*halfwidth + 2*Constants::WALL_HALFWIDTH;
	double maxX = Constants::RIGHT_EDGE - 2*halfwidth - 2*Constants::WALL_HALFWIDTH;
	double randomX = randomInRange(minX, maxX);
	Vector2d pos(randomX, Constants::TOP_EDGE + halfwidth);

	RigidBody* rb = new RigidBody(randomColor(), pos, halfwidth);
	rbs->add(rb);
	rb->applyWrenchW(Vector2d(0,0), tau);
}

void RigidBodySimulation::createWalls()
{
	if (rbs == nullptr)
	{
		printf("Error: rigid body system is null!\n");
		return;
	}

	const double leftEdge = -1;
	const double bottomEdge = -1;
	const double rightEdge = 1;
	const double topEdge = 1;
	Color c(0.f, 0.f, 0.f, 1.f);

	const double epsilon = 0.0001;
	const double wallH = Constants::WALL_HALFWIDTH;

	// floor
	double i = leftEdge + wallH;
	while (i < rightEdge)
	{
		Vector2d pos(i, bottomEdge + wallH);
		RigidBody* rb = new RigidBody(c, pos, wallH - epsilon);
		rb->setPinned(true);
		rbs->add(rb);
		i += 2*wallH;
	}

	// left and right wall
	i = bottomEdge + 3*wallH;
	while (i < topEdge - 2*wallH)
	{
		Vector2d pos(leftEdge + wallH, i);
		RigidBody* rb = new RigidBody(c, pos, wallH - epsilon);
		rb->setPinned(true);
		rbs->add(rb);

		pos.set(rightEdge - wallH, i);
		rb = new RigidBody(c, pos, wallH - epsilon);
		rb->setPinned(true);
		rbs->add(rb);

		i += 2*wallH;
	}
}

Color RigidBodySimulation::randomColor() const
{
	return Color(((double) rand() / RAND_MAX),
			((double) rand() / RAND_MAX),
			((double) rand() / RAND_MAX), 1.f);
}

double RigidBodySimulation::randomInRange(double min, double max) const
{
	return ((double) rand() / RAND_MAX*(max - min)) + min;
}

