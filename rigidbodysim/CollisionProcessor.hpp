//
//  CollisionProcessor.hpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/3/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#ifndef CollisionProcessor_hpp
#define CollisionProcessor_hpp

#include <stdio.h>
#include <unordered_set>
#include <vector>
#include "RigidBody.hpp"
#include "UniformGrid.hpp"

class CollisionProcessor
{
public:
	CollisionProcessor();
	~CollisionProcessor();
	void processCollisions();

	/** Currently we only add bodies.
	 * Call this function to introduce a new body into the system. 
	 */
	void addBody(RigidBody* body);

private:
	struct Contact
	{
		// constructor
		Contact(int iIdx, int jIdx, Vector2d normVec, Vector2d tanVec,
			double aNormI, double aTanI, double aNormJ, double aTanJ)
			: i(iIdx)
			, j(jIdx)
			, normal(normVec)
			, tangent(tanVec)
			, angNormalI(aNormI)
			, angTangentI(aTanI)
			, angNormalJ(aNormJ)
			, angTangentJ(aTanJ)
		{}

		// indices of bodies
		int i, j;
		Vector2d normal;
		Vector2d tangent;
		double angNormalI;	// r_i cross normal
		double angTangentI;	// r_i cross tangent
		double angNormalJ;
		double angTangentJ;
		// scaling factor 1 / D_(i,i) for computing lambda updates
		double dNormalScale = 1.0;
		double dTangentScale = 1.0;
	};

	struct Edge
	{
		Edge(Vector2d maxV, Vector2d f, Vector2d s)
			: maxVertex(maxV)
			, first(f)
			, second(s)
		{}
		
		Vector2d maxVertex;
		Vector2d first;
		Vector2d second;
	};

	/** Broad phase collision test, updates candidate body pairs */
	void broadPhase();
	/** Narrow phase collision detection */
	void narrowPhase();
	/** Helper: Narrow phase collision contact information processing between pairs */
	void processBodyPair(int i, int j);
	/** Collision resolution using constraint solve (Projected Gauss-Seidel).
	 * NOTE: works with the contacts detected and stored by the narrowPhase (processBodyPair) */
	void resolveCollisions();

	/** Helper: store information necessary to reconstruct contact Jacobian 
	 * NOTE: assumes you are passing in a NORMALIZED vector for contact normal!
	 */
	void storeContact(int i, int j, Vector2d contactPoint, Vector2d normal);

	/** Helper: for every contact, precompute b vector values.
	 * NOTE: initializes lambda and deltaLambda vectors as well, given the number of contacts. */
	void computeBVector();

	/** Helper: modifies b vector and contact Jacobian information by
	 * dividing the values by the diagonals of the A matrix. 
	 * NOTE: assumes b vector and contact row info has been computed and stored!
	 */
	void computeDiagonal();

	/** Helper: update lambda (and delta lambda) values. */
	void updateLambdas();

	/** Helper: project (clamp) (next iteration) lambda values to ensure complementarity. */
	double projectLambda(bool frictionConstraint, double lambdaVal) const;

	/** Helper: updates constraint impulse data in rigid body for an input contact.
	 * Assumes relevant lambda has been updated (both normal and tangential). */
	void updateConstraintImpulse(int contactIdx) const;

	/** Returns true if all delta lambda updates are minimal (zero).
	 * Use to test Gauss-Seidel convergence and see if we can exit early. */
	bool hasConverged() const;

	/** MUST be called after final updates to constraint impulses calculated to
	 * prepare for next velocity update. Clears data structures. */
	void resetContacts();

	/** SAT (separating axis theorem) functions */
	/** main collision detection function for SAT, stores contact if detected */
	bool testSATPair(int i, int j);
	// Helper calculation functions
	double axisOverlap(Vector2d currAxis, Vector2d axis1, Vector2d axis2, Vector2d distanceVec) const;

	/** determine contact manifold
	 * From http://www.dyn4j.org/2011/11/contact-points-using-clipping/
	 */
	bool clipPolygons(RigidBody* bodyI, RigidBody* bodyJ, Vector2d normal) const;
	// Calculates clipped points of the edge v1-v0 by (normalized) normal, offset being reference point
	// dotted with the normal. Returns 1-2 points. TODO: should return something less cumbersome than vector
	std::vector<Vector2d> clipEdgeByNormal(Vector2d v0, Vector2d v1, Vector2d normal, double offset) const;
	// Determine best edge to use for polygon clipping, i.e. most perpendicular to separation normal
	Edge getBestEdge(RigidBody* body, Vector2d normal) const;

	// array of bodies
	std::vector<RigidBody*> B;

	// uniform grid for broad phase collision detection
	UniformGrid* G = nullptr;

	/** hash function in order to store body pairs. */
	struct PairHash
	{
		size_t operator()(const std::pair<int, int>& p) const
		{
			return p.first ^ p.second;
		}
	};
	// set of pairs to test for collision
	std::unordered_set<std::pair<int, int>, PairHash> candidateBodyPairs;

	// collision contact calculations
	std::vector<Contact> contacts;
	// map of a list of contacts for a body i
	//std::unordered_map<int, std::vector<int>> contactsForBody;
	// lambda vector (constraint impulse magnitudes)
	std::vector<double> lambdas;
	// delta lambda vector (change in constraint impulse magnitudes)
	std::vector<double> deltaLambdas;
	// b vector
	std::vector<double> bValues;
};

#endif /* CollisionProcessor_hpp */
