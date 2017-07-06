//
//  main.cpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/1/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#include <vector>
#include <stdio.h>

#include "RigidBodySimulation.hpp"
	
int main( int argc, char* args[] )
{
	// std::vector<Vector2d> positions;
	// positions.push_back(Vector2d(0.05, 0.0));
	// positions.push_back(Vector2d(0.6, 0.6));
	// positions.push_back(Vector2d(0.2, -0.25));
	// positions.push_back(Vector2d(-0.4, -0.3));
	// positions.push_back(Vector2d(-0.4, 0.3));
	// positions.push_back(Vector2d(0.02, 0.7));

	RigidBodySimulation* rbs = new RigidBodySimulation();
	// rbs->setScene(positions, 0.05f);
	rbs->start();

	// main loop complete
	delete rbs;
	return 0;
}