//
//  UniformGrid.hpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/13/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#ifndef UniformGrid_hpp
#define UniformGrid_hpp

#include <stdio.h>
#include <vector>
#include "RigidBody.hpp"

class UniformGrid
{
public:
	UniformGrid();
	UniformGrid(int gridRows);

	struct GridCell
	{
		std::vector<int> bodies;
	};

	/** Determines grid index of given body. */
	void assignBodyToGrid(const RigidBody* body);

	/** Getter for grid. */
	const std::vector<GridCell>& getGrid() const;

	/** Clears grid body indices. */
	void resetGrid();

private:
	/** Builds a grid of gridRows*gridRows size. */
	void buildGrid(int gridRows);

	/** Helper: maps a coordinate (x or y) to the appropriate grid row or col. */
	int mapToGrid(double input, bool isX) const;

	int gridRows = 4;
	/** number of boxes in which to divide area, must be square number.
	 * NOTE: this is the number of sets we store in our grid! */
	int subdivisions = 16;

	// total coordinate ranges
	double xRange = 1;
	double yRange = 1;

	/** grid object, containing collision indices of bodies (in collision processor) */
	std::vector<GridCell> grid;
};

#endif /* UniformGrid_hpp */
