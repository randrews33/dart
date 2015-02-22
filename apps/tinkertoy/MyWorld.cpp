#include "MyWorld.h"
#include "Particle.h"
#include <iostream>

#define GRAVITY 9.81

using namespace Eigen;

MyWorld::MyWorld() {
	// Create particles
	Particle *p1 = new Particle();
	Particle *p2 = new Particle();

	// First two particles are constrained to each other, so set their flags accordingly
	p1->mConstrained = true;
	p2->mConstrained = true;

	mParticles.push_back(p1);
	mParticles.push_back(p2);

	// Init particle position
	mParticles[0]->mPosition[0] = 0.2;
	mParticles[1]->mPosition[0] = 0.2;
	mParticles[1]->mPosition[1] = -0.1;

	// DEBUG
	std::cout << "Particle vector size: " << mParticles.size() << std::endl;

	// Value here is based on initializations above
	double distance = sqrt((mParticles[1]->mPosition - mParticles[0]->mPosition).dot(
		mParticles[1]->mPosition - mParticles[0]->mPosition));
	distances.push_back(distance);

	J.resize(2, 6);
	J.setZero();

	dJ.resize(2, 6);
	dJ.setZero();

	q.resize(6, 1);
	dq.resize(6, 1);
	
	W.resize(6, 6);
	W.setZero();
	W.block<3, 3>(0, 0) = Matrix3d::Identity() / mParticles[0]->mMass;
	W.block<3, 3>(3, 3) = Matrix3d::Identity() / mParticles[1]->mMass;

	Q.resize(6, 1);
	lambda.resize(2);
	Qh.resize(6, 1);
	C.resize(2, 1);
	dC.resize(2, 1);

	nConstraints = 2;
	constraints.push_back(new CircleConstraint(mParticles[0], 0.2, Vector3d(0.0, 0.0, 0.0)));
	constraints.push_back(new DistanceConstraint(mParticles[1], mParticles[0]));
}

MyWorld::~MyWorld() {
	for (int i = 0; i < mParticles.size(); i++)
		delete mParticles[i];
	mParticles.clear();
}

int MyWorld::getNumParticles() {
	return mParticles.size();
}

void MyWorld::addParticle(Vector3d position) {
	Particle *p = new Particle();
	p->mPosition = position;
	mParticles.push_back(p);
	std::cout << "Added new particle at (" << position[0] << "," << position[1] << "," << position[2] << ")" << std::endl;

	// Since current assumption is that each new particle will operate under a distance-based constraint from the previous
	// particle, push its distance from the previous particle into the distances vector
	double distance = sqrt((mParticles[mParticles.size() - 1]->mPosition - mParticles[mParticles.size() - 2]->mPosition).dot(
		mParticles[mParticles.size() - 1]->mPosition - mParticles[mParticles.size() - 2]->mPosition));
	distances.push_back(distance);

	// Resize & initialize constraint parameter matrices
	q.resize(q.rows() + 3, 1);
	dq.resize(dq.rows() + 3, 1);
	Q.resize(Q.rows() + 3);
	W.resize(mParticles.size() * 3, mParticles.size() * 3);
	Qh.resize(Qh.rows() + 3, 1);

	J.resize(J.rows(), J.cols() + 3);
	dJ.resize(dJ.rows(), dJ.cols() + 3);

	// Currently under assumption that any new particle will have a distance-based constraint on the previous particle in the array
	mParticles[mParticles.size() - 1]->mConstrained = true;
	mParticles[mParticles.size() - 2]->mConstrained = true;
	addConstraint(new DistanceConstraint(mParticles[mParticles.size() - 2], mParticles[mParticles.size() - 1]));

	initMats();
}

void MyWorld::addConstraint(Constraint* constraint) {
	C.resize(C.rows() + 1, 1);
	dC.resize(dC.rows() + 1, 1);
	J.resize(J.rows() + 1, J.cols());
	dJ.resize(dJ.rows() + 1, dJ.cols());
	lambda.resize(lambda.rows() + 1, 1);

	constraints.push_back(constraint);
	nConstraints++;
}

void MyWorld::simulate() {
	// Replace the following code
	/*
	for (int i = 0; i < mParticles.size(); i++)
	mParticles[i]->mPosition[1] -= 0.005;
	*/
	float tStep = 0.002;

	updateForces();
	updateConstraintParams();
	calculateConstraints();
	applyConstraints();
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->update(tStep);
	}
	resetForces();
}

void MyWorld::updateForces() {
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mAccumulatedForce += Vector3d(0, -1, 0) * GRAVITY;
	}
}

void MyWorld::resetForces() {
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mAccumulatedForce.setZero();
	}
}

void MyWorld::updateConstraintParams() {
	// Operate under assumption that first particle is the only one operating under a circle constraint
	// It will also be assumed that the rest of the particles will be constrained to a specific distance from the previous one

	for (int i = 0; i < nConstraints; i++) {
		for (int j = 0; j < mParticles.size(); j++) {
			J.block<1, 3>(i, j * 3) = constraints[i]->J(mParticles[j]);
			dJ.block<1, 3>(i, j * 3) = constraints[i]->dJ(mParticles[j]);
		}
		C(i, 0) = constraints[i]->C();
		dC(i, 0) = constraints[i]->dC();
	}

	for (int i = 0; i < mParticles.size(); i++) {
		Q.block<3, 1>(i * 3, 0) = mParticles[i]->mAccumulatedForce;
		q.block<3, 1>(i * 3, 0) = mParticles[i]->mPosition;
		dq.block<3, 1>(i * 3, 0) = mParticles[i]->mVelocity;
	}

	//print_matrices();
}

void MyWorld::calculateConstraints() {
	double ks = 20.0, kd = 5.0;
	lambda = (J * W * J.transpose()).inverse() * (-dJ * dq - J * W * Q - ks * C - kd * dC);
	Qh = J.transpose() * lambda;
	//static int fCount = 0;
	//std::cout << "[ " << fCount++ << " ] " << std::endl << Qh << std::endl << "------------" << std::endl;
}

void MyWorld::applyConstraints() {
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mAccumulatedForce += Qh.block<3, 1>(i*3, 0);
	}
}

// Use this to re-initialize matrices when adding/removing particles or constraints
void MyWorld::initMats() {
	// Initialize W by first setting everything to 0, and then setting diagonal
	for (int i = 0; i < mParticles.size(); i++) {
		q.block<3, 1>(i * 3, 0) = mParticles[i]->mPosition;
		dq.block<3, 1>(i * 3, 0) = mParticles[i]->mVelocity;
		Q.block<3, 1>(i * 3, 0) = mParticles[i]->mAccumulatedForce;
	}

	J.setZero();
	dJ.setZero();

	// Set diagonal of W
	W.setZero();
	for (int i = 0; i < mParticles.size(); i++) {
		W.block<3, 3>(i * 3, i * 3) = Matrix3d::Identity() / mParticles[i]->mMass;
	}
}