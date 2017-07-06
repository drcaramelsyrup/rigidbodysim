//
//  UniformGrid.cpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/13/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#include <cmath>
#include "UniformGrid.hpp"
#include "Utils.h"

UniformGrid::UniformGrid()
	: gridRows(4)
{
	buildGrid(gridRows);	// default is 4.
}

UniformGrid::UniformGrid(int gridSections)
	: gridRows(gridSections)
{
	buildGrid(gridSections);
}

const std::vector<UniformGrid::GridCell>& UniformGrid::getGrid() const
{
	return grid;
}

void UniformGrid::resetGrid()
{
	for (GridCell& cell : grid)
	{
		cell.bodies.clear();
	}
}

void UniformGrid::buildGrid(int gridSections)
{
	xRange = (double)Constants::RIGHT_EDGE - Constants::LEFT_EDGE;
	yRange = (double)Constants::TOP_EDGE - Constants::BOTTOM_EDGE;

	// prepare grid allocation
	gridRows = gridSections;
	subdivisions = gridSections*gridSections;
	grid.resize(subdivisions);
}

void UniformGrid::assignBodyToGrid(const RigidBody* rb)
{
	Vector2d p = rb->getPosition();
	double radius = rb->getRadius();
	int minXIdx = (p.x - radius);
	int minYIdx = (p.y - radius);
	int maxXIdx = (p.x + radius);
	int maxYIdx = (p.y + radius);

	// map onto appropriate range
	int minRow = mapToGrid(minXIdx, true);
	int maxRow = mapToGrid(maxXIdx, true);
	int minCol = mapToGrid(minYIdx, false);
	int maxCol = mapToGrid(maxYIdx, false);

	for (int r = minRow; r <= maxRow; ++r)
	{
		for (int c = minCol; c <= maxCol; ++c)
		{
			// block might be there, but it's wholly out of bounds.
			if (r*c > subdivisions - 1 || r*c < 0)
			{
				printf("Warning: uniform grid did not assign body to out of bounds cell!\n");
				return;
			}
			grid[r*c].bodies.push_back(rb->colIndex);
		}
	}
}

int UniformGrid::mapToGrid(double input, bool isX) const
{
	int output = 0;
	// truncate values
	if (isX)
	{
		output = (int)(((input - Constants::LEFT_EDGE) * gridRows) / xRange);
	}
	else
	{
		output = (int)(((input - Constants::BOTTOM_EDGE) * gridRows) / yRange);
	}
	// if beyond threshold, assign to last bucket
	return output == gridRows ? gridRows - 1 : output;
}