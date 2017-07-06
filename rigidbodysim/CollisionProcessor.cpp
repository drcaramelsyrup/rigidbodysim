//
//  CollisionProcessor.cpp
//  rigidbodysim
//
//  Created by Edward Wang on 12/3/16.
//  Copyright © 2016 Edward Wang. All rights reserved.
//

#include "CollisionProcessor.hpp"

CollisionProcessor::CollisionProcessor()
{
	if (G == nullptr)
		G = new UniformGrid();
}

CollisionProcessor::~CollisionProcessor()
{
	if (G != nullptr)
		delete G;
	G = nullptr;
}

void CollisionProcessor::processCollisions()
{
	resetContacts();
	broadPhase();
	narrowPhase();
	resolveCollisions();
}

void CollisionProcessor::addBody(RigidBody* body)
{
	if (body == nullptr)
	{
		printf("Error: body to add to CollisionProcessor is null!\n");
		return;
	}

	int bodyIdx = B.size();
	B.push_back(body);
	body->colIndex = bodyIdx;
}

void CollisionProcessor::broadPhase()
{
	// uniform grid
	// update candidate body pairs to test for collision

	// map bodies to grid cells
	for (RigidBody* body : B)
	{
		G->assignBodyToGrid(body);
	}

	// for each grid cell, make candidate pairs from the bodies inside the cell
	const auto& grid = G->getGrid();
	for (const auto& cell : grid)
	{
		const auto& bodies = cell.bodies;
		for (int i = 0; i < bodies.size(); ++i)
		{
			for (int j = 0; j < i; ++j)
			{
				if (i == j) continue;
				candidateBodyPairs.insert(std::make_pair(i, j));				
			}
		}
	}

	// all pairs test
	// for (int i = 0; i < B.size(); ++i)
	// {
	// 	for (int j = 0; j < i; ++j)
	// 	{
	// 		if (i == j) continue;
	// 		candidateBodyPairs.insert(std::make_pair(i, j));
	// 	}
	// }
}

void CollisionProcessor::narrowPhase()
{
	// block boundary test.
	for (const auto& bodyPair : candidateBodyPairs)
	{
		//processBodyPair(bodyPair.first, bodyPair.second);
		// SAT detection not quite working...
		testSATPair(bodyPair.first, bodyPair.second);
	}

}

void CollisionProcessor::resolveCollisions()
{
	// precomputation
	computeBVector();
	computeDiagonal();

	for (int k = 0; k < Constants::SOLVER_ITERATIONS; ++k)
	{
		updateLambdas();
		if (hasConverged())
			break;
	}
}

void CollisionProcessor::storeContact(int i, int j, Vector2d contactPoint, Vector2d normal)
{
	// unit contact tangent vector
	Vector2d tangentVec;
	// rotate counterclockwise 90 degrees
	tangentVec.x = -normal.y;
	tangentVec.y = normal.x;
	// p_i - x_i
	Vector2d rI(contactPoint);
	rI.sub(B[i]->getPosition());
	// p_j - x_j
	Vector2d rJ(contactPoint);
	rJ.sub(B[j]->getPosition());

	// construct Contact struct inside array
	contacts.emplace_back(
		i, j,
		normal, 
		tangentVec, 
		Vector2d::cross(rI, normal),
		Vector2d::cross(rI, tangentVec),
		Vector2d::cross(rJ, normal),
		Vector2d::cross(rJ, tangentVec));
}

void CollisionProcessor::computeBVector()
{
	for (const Contact& contact : contacts)
	{
		RigidBody* bodyI = B[contact.i];
		RigidBody* bodyJ = B[contact.j];
		if (bodyI == nullptr || bodyJ == nullptr)
		{
			printf("Error: invalid body pointers in collision processor!\n");
			return;
		}

		Vector2d linearI(bodyI->getVelocity());
		double angularI = bodyI->getVelocityAngular();
		Vector2d linearJ(bodyJ->getVelocity());
		double angularJ = bodyJ->getVelocityAngular();

		// update according to foreseen external forces/torques
		linearI.acc(Constants::DT*bodyI->getInvMass(), bodyI->getForce());
		angularI += (Constants::DT*bodyI->getInvMassAngular(), bodyI->getTorque());

		linearJ.acc(Constants::DT*bodyJ->getInvMass(), bodyJ->getForce());
		angularJ += (Constants::DT*bodyJ->getInvMassAngular(), bodyJ->getTorque());

		// b vector values
		// negate body i's end results
		double impulseI = -Vector2d::dot(contact.normal, linearI);
		impulseI += (-contact.angNormalI * angularI);
		double impulseJ = Vector2d::dot(contact.normal, linearJ);
		impulseJ += (contact.angNormalJ * angularJ);

		double frictionI = -Vector2d::dot(contact.tangent, linearI);
		frictionI += (-contact.angTangentI * angularI);
		double frictionJ = Vector2d::dot(contact.tangent, linearJ);
		frictionJ += (contact.angTangentJ * angularJ);

		bValues.push_back(impulseI + impulseJ);
		bValues.push_back(frictionI + frictionJ);

		// initialization of lambda vector values
		lambdas.push_back(0.0); lambdas.push_back(0.0);
		deltaLambdas.push_back(0.0); deltaLambdas.push_back(0.0);

	}
}

void CollisionProcessor::computeDiagonal()
{
	// calculate diagonal elements of A matrix
	for (int i = 0; i < contacts.size(); ++i)
	{
		// contact Jacobian information to be modified into J row prime
		Contact& contact = contacts[i];

		const RigidBody* bodyI = B[contact.i];
		const RigidBody* bodyJ = B[contact.j];
		if (bodyI == nullptr || bodyJ == nullptr)
		{
			printf("Error: invalid rigid body pointers in collision processor!\n");
			return;
		}

		// preparing inv mass matrix info
		double invMassI = bodyI->getInvMass();
		double invMassJ = bodyJ->getInvMass();
		double invInertiaI = bodyI->getInvMassAngular();
		double invInertiaJ = bodyJ->getInvMassAngular();

		// J * invM * J^T
		// normal impulse
		Vector2d scaledNormalI(contact.normal);
		scaledNormalI.scale(invMassI);
		Vector2d scaledNormalJ(contact.normal);
		scaledNormalJ.scale(invMassJ);
		double DNormal =
			scaledNormalI.lengthSq()	// dot product with itself
			+ scaledNormalJ.lengthSq()
			+ invInertiaI * contact.angNormalI*contact.angNormalI
			+ invInertiaJ * contact.angNormalJ*contact.angNormalJ;
		// friction
		Vector2d scaledTangentI(contact.tangent);
		scaledTangentI.scale(invMassI);
		Vector2d scaledTangentJ(contact.tangent);
		scaledTangentJ.scale(invMassJ);
		double DTangent =
			scaledTangentI.lengthSq()
			+ scaledTangentJ.lengthSq()
			+ invInertiaI * contact.angTangentI*contact.angTangentI
			+ invInertiaJ * contact.angTangentJ*contact.angTangentJ;

		// being careful not to divide by zero.
		// this really shouldn't happen, but...
		if (DNormal == 0.0)
		{
			printf("Error: A matrix diagonal zero! added epsilon to diagonal values: bodies %d and %d\n",
				contact.i, contact.j);
			DNormal += Constants::DISTANCE_EPSILON;
		}
		if (DTangent == 0.0)
		{
			printf("Error: A matrix diagonal zero! added epsilon to diagonal values: bodies %d and %d\n",
				contact.i, contact.j);
			DTangent += Constants::DISTANCE_EPSILON;
		}	

		// scale (two) rows by D_i,i
		contact.dNormalScale = 1.0 / DNormal;
		contact.dTangentScale = 1.0 / DTangent;

		// modify b vector into precomputed b prime vector
		bValues[2*i] /= DNormal;
		bValues[2*i+1] /= DTangent;
	}
}

void CollisionProcessor::updateLambdas()
{
	int idx = 0;
	for (const Contact& contact : contacts)
	{
		// normal and tangent row indices.
		int nIdx = 2*idx;
		int tIdx = 2*idx + 1;

		RigidBody* bodyI = B[contact.i];
		RigidBody* bodyJ = B[contact.j];
		if (bodyI == nullptr || bodyJ == nullptr)
		{
			printf("Error: invalid body pointers in collision processor!\n");
			return;
		}

		/** normal */
		double deltaLambda = -bValues[nIdx];
		// scale by D_(i,i)
		Vector2d normalPrime(contact.normal);
		normalPrime.scale(contact.dNormalScale);
		// i (negate)
		deltaLambda += Vector2d::dot(normalPrime, bodyI->deltaLinV);
		deltaLambda += (contact.angNormalI * contact.dNormalScale) * bodyI->deltaAngV;
		// j
		deltaLambda -= Vector2d::dot(normalPrime, bodyJ->deltaLinV);
		deltaLambda -= (contact.angNormalJ * contact.dNormalScale) * bodyJ->deltaAngV;

		/** friction */
		double deltaLambdaFriction = -bValues[tIdx];
		// scale by D_(i,i)
		Vector2d tangentPrime(contact.tangent);
		tangentPrime.scale(contact.dTangentScale);
		// i (negate)
		deltaLambdaFriction += Vector2d::dot(tangentPrime, bodyI->deltaLinV);
		deltaLambdaFriction += (contact.angTangentI * contact.dTangentScale) * bodyI->deltaAngV;
		// j
		deltaLambdaFriction -= Vector2d::dot(tangentPrime, bodyJ->deltaLinV);
		deltaLambdaFriction -= (contact.angTangentJ * contact.dTangentScale) * bodyJ->deltaAngV;

		/** final array updates */
		double oldLambda = lambdas[nIdx];
		double oldLambdaFriction = lambdas[tIdx];
		// project final lambda values
		lambdas[nIdx] = projectLambda(false, lambdas[nIdx] + deltaLambda);	// normal
		lambdas[tIdx] = 0;
		// lambdas[tIdx] = projectLambda(true, lambdas[tIdx] + deltaLambdaFriction);	// friction

		// deltas
		deltaLambdas[nIdx] = lambdas[nIdx] - oldLambda;
		deltaLambdas[tIdx] = 0;
		// deltaLambdas[tIdx] = lambdas[tIdx] - oldLambdaFriction;

		// update deltaV immediately
		updateConstraintImpulse(idx);

		// increment storage index
		++idx;
	}
}

double CollisionProcessor::projectLambda(bool friction, double lambdaVal) const
{
	// lambaVal = lambda (k+1 iteration)
	if (!friction)
	{
		// constraint impulse
		return lambdaVal >= 0.0 ? lambdaVal : 0.0;
	}

	// else friction constraint
	double lambdaHi = Constants::FRICTION_COEFFICIENT * lambdaVal;
	double lambdaLo = -lambdaHi;
	double finalLambda = (lambdaLo > lambdaVal) ? lambdaLo : lambdaVal;	// max
	finalLambda = (lambdaHi < finalLambda) ? lambdaHi : finalLambda;	// min

	return finalLambda;
}

void CollisionProcessor::updateConstraintImpulse(int cIdx) const
{
	// normal and tangent row indices
	int nIdx = 2*cIdx;
	int tIdx = 2*cIdx + 1;

	const Contact& contact = contacts[cIdx];
	RigidBody* bodyI = B[contact.i];
	RigidBody* bodyJ = B[contact.j];
	if (bodyI == nullptr || bodyJ == nullptr)
	{
		printf("Error: invalid body pointers in collision processor!\n");
		return;
	}

	// lambdas for contact index cIdx
	double dLambda = deltaLambdas[nIdx];
	double dLambdaFriction = deltaLambdas[tIdx];
	// determine sign of Jacobian
	// deltaV contraint impulse updates (deltaV + invMass * Jacobian * deltaLambda)
	// i (negate)
	bodyI->deltaLinV.acc(-dLambda*bodyI->getInvMass(), contact.normal);
	bodyI->deltaLinV.acc(-dLambdaFriction*bodyI->getInvMass(), contact.tangent);
	bodyI->deltaAngV -=
		(contact.angNormalI*bodyI->getInvMassAngular() * dLambda
		+ contact.angTangentI*bodyI->getInvMassAngular() * dLambdaFriction);
	// j
	bodyJ->deltaLinV.acc(dLambda*bodyJ->getInvMass(), contact.normal);
	bodyJ->deltaLinV.acc(dLambdaFriction*bodyJ->getInvMass(), contact.tangent);
	bodyJ->deltaAngV +=
		(contact.angNormalJ*bodyJ->getInvMassAngular() * dLambda
		+ contact.angTangentJ*bodyJ->getInvMassAngular() * dLambdaFriction);
}

bool CollisionProcessor::hasConverged() const
{
	for (double deltaLambda : deltaLambdas)
	{
		if (deltaLambda > Constants::CONVERGE_EPSILON 
			|| deltaLambda < -Constants::CONVERGE_EPSILON)
			return false;
	}
	return true;
}

void CollisionProcessor::resetContacts()
{
	G->resetGrid();
	candidateBodyPairs.clear();
	contacts.clear();
	bValues.clear();
	lambdas.clear();
	deltaLambdas.clear();
}

void CollisionProcessor::processBodyPair(int i, int j)
{
	RigidBody* bodyI = B[i];
	RigidBody* bodyJ = B[j];

	if (bodyI == nullptr || bodyJ == nullptr)
	{
		printf("Error: invalid body pointers in collision processor!\n");
		return;
	}

	// omit detection of contacts where both blocks are pinned.
	if (bodyI->isPinned() && bodyJ->isPinned())
		return;

	// simple block test, since our rigid bodies are currently just a block.
	Vector2d bodyIWorldPos(bodyI->getPosition());
	Vector2d bodyJWorldPos(bodyJ->getPosition());

	// bodyJWorldPos - bodyIWorldPos
	Vector2d v;
	v.set(bodyJWorldPos);
	v.sub(bodyIWorldPos);
	double sumRadii = bodyI->getH() + bodyJ->getH();

	// contact test
	if (std::abs(v.x) < sumRadii && std::abs(v.y) < sumRadii)
	{
		// calculating the contact point. should be midpoint
		// of line connecting center of bodies
		Vector2d contactPoint(bodyIWorldPos.x + v.x/2.0, bodyIWorldPos.y + v.y/2.0);

		v.normalize();

		storeContact(i, j, contactPoint, v);
	}
}

bool CollisionProcessor::testSATPair(int i, int j)
{
	RigidBody* bodyI = B[i];
	RigidBody* bodyJ = B[j];
	if (bodyI == nullptr || bodyJ == nullptr)
	{
		printf("Error: invalid body pointers in collision processor!\n");
		return false;
	}

	// omit detection of contacts where both blocks are pinned.
	if (bodyI->isPinned() && bodyJ->isPinned())
		return false;

	Vector2d iPos = bodyI->getPosition();
	Vector2d jPos = bodyJ->getPosition();
	// double iH = bodyI->getH();
	// double jH = bodyJ->getH();

	// bodyJWorldPos - bodyIWorldPos, distance from centers
	Vector2d v;
	v.set(jPos);
	v.sub(iPos);

	// // half width vectors (separating axes)
	// // i
	// Vector2d iWVec(iH, 0);
	// Vector2d iHVec(0, iH);
	// iWVec = bodyI->rotateByOrientation(iWVec);
	// iHVec = bodyI->rotateByOrientation(iHVec);
	// // bodyI->setAxes(iWVec, iHVec);

	// // j
	// Vector2d jWVec(jH, 0);
	// Vector2d jHVec(0, jH);
	// jWVec = bodyJ->rotateByOrientation(jWVec);
	// jHVec = bodyJ->rotateByOrientation(jHVec);
	// // bodyJ->setAxes(jWVec, jHVec);

	// i
	Vector2d iWVec = bodyI->getAxis1();
	Vector2d iHVec = bodyI->getAxis2();
	// j
	Vector2d jWVec = bodyJ->getAxis1();
	Vector2d jHVec = bodyJ->getAxis2();

	double overlap = 0;
	double smallestOverlap = 0;
	Vector2d smallestAxis;
	Vector2d axis;

	// determining order of rigid bodies for collision processing
	int refBody = i;
	int collidingBody = j;

	// test i axes
	// w_i
	if ((overlap = axisOverlap(iWVec, jWVec, jHVec, v)) < 0)
		return false;
	smallestOverlap = overlap;
	smallestAxis.set(iWVec);

	// h_i
	if ((overlap = axisOverlap(iHVec, jWVec, jHVec, v)) < 0)
		return false;
	if (smallestOverlap > overlap)
	{
		smallestOverlap = overlap;
		smallestAxis.set(iHVec);
	}

	// test j axes
	// w_j
	if ((overlap = axisOverlap(jWVec, iWVec, iHVec, v)) < 0)
		return false;
	if (smallestOverlap > overlap)
	{
		smallestOverlap = overlap;
		smallestAxis.set(jWVec);
		// switch order of body pair
		refBody = j;
		collidingBody = i;
	}

	// h_j
	if ((overlap = axisOverlap(jHVec, iWVec, iHVec, v)) < 0)
		return false;
	if (smallestOverlap > overlap)
	{
		smallestOverlap = overlap;
		smallestAxis.set(jHVec);
		refBody = j;
		collidingBody = i;
	}

	// still here? collision! store contact point
	// double penDepth = smallestOverlap;
	smallestAxis.normalize();
	Vector2d contactPoint;
	// set contact point as mid point for now
	contactPoint.set(iPos.x + v.x/2.0, iPos.y + v.y/2.0);
	// contactPoint.set(origin.x + smallestAxis.x*penDepth, origin.y + smallestAxis.y*penDepth);

	// save contact normal as smallest separating axis
	// storeContact(refBody, collidingBody, contactPoint, smallestAxis);
	// save contact normal as normalized distance vector
	v.normalize();
	storeContact(i, j, contactPoint, v);
	return true;
}

double CollisionProcessor::axisOverlap(Vector2d currAxis, 
	Vector2d axis1, Vector2d axis2, Vector2d distanceVec) const
{
	distanceVec.projectOnto(currAxis);
	axis1.projectOnto(currAxis);
	axis2.projectOnto(currAxis);
	return currAxis.length() + axis1.length() + axis2.length() - distanceVec.length();
}

bool CollisionProcessor::clipPolygons(RigidBody* bodyI, RigidBody* bodyJ, Vector2d normal) const
{
	if (bodyI == nullptr || bodyJ == nullptr)
	{
		printf("Error: invalid body pointers in collision processor!\n");
		return false;
	}

	Edge bestI = getBestEdge(bodyI, normal);
	Edge bestJ = getBestEdge(bodyJ, normal.negated());
	Edge refEdge = bestI;
	Edge incEdge = bestJ;

	if (std::abs(Vector2d::dot(bestI.second - bestI.first, normal)) 
		> std::abs(Vector2d::dot(bestJ.second - bestJ.first, normal)))
	{
		refEdge = bestJ;
		incEdge = bestI;
		// flipped is true
	}



	// most negative dot product to separation normal. remember to flip normal for i
	// determine edges, then reference edge and incident edge
	// clip incident edge with reference edge

	return true;

}

std::vector<Vector2d> CollisionProcessor::clipEdgeByNormal(Vector2d v0, Vector2d v1, Vector2d normal, double offset) const
{
	std::vector<Vector2d> clippedPoints;

	double d0 = Vector2d::dot(v0, normal) - offset;
	double d1 = Vector2d::dot(v1, normal) - offset;

	if (d0 >= 0.0)
		clippedPoints.push_back(v0);
	if (d1 >= 0.0)
		clippedPoints.push_back(v1);

	// something (one point) must be clipped!
	// if both are clipped, then we know something is up and do not calculate the collision.
	if (d0 * d1 < 0.0)
	{
		Vector2d edge = v1 - v0;
		// location along edge
		double ratio = d0 / (d0 - d1);
		edge.scale(ratio);
		edge.add(v0);
		clippedPoints.push_back(edge);
	}

	return clippedPoints;
}

CollisionProcessor::Edge CollisionProcessor::getBestEdge(RigidBody* body, Vector2d normal) const
{
	std::vector<Vector2d> vertices = body->getVertices();

	double maxSimilarity = 0;
	int maxIdx = -1;
	// which vertex is farthest along separation normal?
	for (int i = 0; i < vertices.size(); ++i)
	{
		double similarity = Vector2d::dot(vertices[i], normal);
		if (similarity > maxSimilarity || maxIdx == -1)
		{
			maxSimilarity = similarity;
			maxIdx = i;
		}
	}

	// left or right edge?
	int prev = maxIdx - 1 >= 0 ? maxIdx - 1 : vertices.size() - 1;	// wraparound
	int next = maxIdx + 1 < vertices.size() ? maxIdx + 1 : 0;

	Vector2d e0 = vertices[maxIdx] - vertices[prev];
	e0.normalize();
	Vector2d e1 = vertices[maxIdx] - vertices[next];
	e1.normalize();

	if (Vector2d::dot(e0, normal) <= Vector2d::dot(e1, normal))
	{
		// use e0
		return Edge(vertices[maxIdx], vertices[prev], vertices[maxIdx]);
	}
	// use e1
	return Edge(vertices[maxIdx], vertices[maxIdx], vertices[next]);
}




