#include "MyWorld.h"
#include "Particle.h"
#include <iostream>

#define GRAVITY 9.81

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) {
	// Create particles
	for (int i = 0; i < _numParticles; i++) {
		Particle *p1 = new Particle();
		Particle *p2 = new Particle();
		mParticles.push_back(p1);
		mParticles.push_back(p2);
	}

	// Init particle position
	mParticles[0]->mPosition[0] = 0.2;
	mParticles[1]->mPosition[0] = 0.2;
	mParticles[1]->mPosition[1] = -0.1;

	// Value here is based on initializations above
	distances.push_back(0.1);

	J.resize(2, 6);
	J.setZero();

	dJ.resize(2, 6);
	dJ.setZero();

	q.resize(6, 1);
	q.block<3, 1>(0, 0) = mParticles[0]->mPosition;
	q.block<3, 1>(3, 0) = mParticles[1]->mPosition;

	dq.resize(6, 1);
	dq.block<3, 1>(0, 0) = mParticles[0]->mVelocity;
	dq.block<3, 1>(3, 0) = mParticles[1]->mVelocity;
	
	W.resize(6, 6);
	W.setZero();
	W.block<3, 3>(0, 0) = Matrix3d::Identity() / mParticles[0]->mMass;
	W.block<3, 3>(3, 3) = Matrix3d::Identity() / mParticles[1]->mMass;

	Q.resize(6, 1);
	lambda.resize(2);
	Qh.resize(6, 1);

	C.resize(2, 1);
	C(0, 0) = 0.5 * mParticles[0]->mPosition.dot(mParticles[0]->mPosition) - 0.5 * 0.2 * 0.2;
	C(1, 0) = 0.5 * mParticles[0]->mPosition.dot(mParticles[0]->mPosition) - mParticles[0]->mPosition.dot(mParticles[1]->mPosition) +
		0.5 * mParticles[1]->mPosition.dot(mParticles[1]->mPosition) - 0.5 * 0.1 * 0.1;

	dC.resize(2, 1);
	dC(0, 0) = mParticles[0]->mPosition.dot(mParticles[0]->mVelocity);
	dC(1, 0) = mParticles[0]->mPosition.dot(mParticles[0]->mVelocity) -
		mParticles[0]->mPosition.dot(mParticles[1]->mVelocity) -
		mParticles[1]->mPosition.dot(mParticles[0]->mVelocity) +
		mParticles[1]->mPosition.dot(mParticles[1]->mVelocity);

	nConstraints = 2;
}

MyWorld::~MyWorld() {
	for (int i = 0; i < mParticles.size(); i++)
		delete mParticles[i];
	mParticles.clear();
}

void MyWorld::addParticle(Vector3d position) {
	Particle *p = new Particle();
	p->mPosition = position;
	mParticles.push_back(p);
	std::cout << "Added new particle at (" << position[0] << "," << position[1] << "," << position[2] << ")" << std::endl;

	// Since current assumption is that each new particle will operate under a distance-based constraint from the previous
	// particle, push its distance from the previous particle into the distances vector
	float distance = sqrt((mParticles[mParticles.size() - 1]->mPosition - mParticles[mParticles.size() - 2]->mPosition).dot(
		mParticles[mParticles.size() - 1]->mPosition - mParticles[mParticles.size() - 2]->mPosition));
	distances.push_back(distance);

	// Resize & initialize constraint parameter matrices
	q.resize(q.rows() + 3, 1);
	//q.block<3, 1>(q.rows() - 3, 0) = position;

	dq.resize(dq.rows() + 3, 1);
	//dq.block<3, 1>(dq.rows() - 3, 0) = Vector3d::Zero();

	Q.resize(Q.rows() + 3);
	//Q.block<3, 1>(Q.rows() - 3, 0) = Vector3d::Zero();

	// For this initialization of W, just going to assume that newly added elements are initialized to 0.  Will verify
	// EDIT: Newly added elements ARE NOT INITIALIZED.  In fact, ALL ELEMENTS LOSE THEIR VALUES AND MUST BE RE-INITIALIZED
	W.resize(mParticles.size() * 3, mParticles.size() * 3);
	//W.block<3, 3>(mParticles.size() * 3 - 3, mParticles.size() * 3 - 3) = Matrix3d::Identity() / mParticles[mParticles.size() - 1]->mMass;

	// Currently under assumption that any new particle will have a distance-based constraint on the previous particle in the array
	C.resize(C.rows() + 1, 1);
	dC.resize(dC.rows() + 1, 1);

	J.resize(J.rows() + 1, J.cols() + 3);
	//J.block<1, 3>(J.rows() - 1, J.cols() - 3) = Vector3d::Zero();
	//for (int i = 0; i < nConstraints - 1; i++) J.block<1, 3>(i, mParticles.size() * 3 - 3) = Vector3d::Zero();

	dJ.resize(dJ.rows() + 1, dJ.cols() + 3);
	//dJ.block<1, 3>(dJ.rows() - 1, dJ.cols() - 3) = Vector3d::Zero();
	//for (int i = 0; i < nConstraints - 1; i++) dJ.block<1, 3>(i, mParticles.size() * 3 - 3) = Vector3d::Zero();

	lambda.resize(lambda.rows() + 1, 1);
	Qh.resize(Qh.rows() + 3, 1);

	nConstraints++;

	initMats();
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
}

void MyWorld::updateForces() {
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mAccumulatedForce.setZero();
		mParticles[i]->mAccumulatedForce += Vector3d(0, -1, 0) * GRAVITY;
	}
}

void MyWorld::updateConstraintParams() {
	// Operate under assumption that first particle is the only one operating under a circle constraint
	// It will also be assumed that the rest of the particles will be constrained to a specific distance from the previous one

	C(0, 0) = 0.5 * mParticles[0]->mPosition.dot(mParticles[0]->mPosition) - 0.5 * 0.2 * 0.2;
	dC(0, 0) = mParticles[0]->mPosition.dot(mParticles[0]->mVelocity);

	J.block<1, 3>(0, 0) = mParticles[0]->mPosition;
	dJ.block<1, 3>(0, 0) = mParticles[0]->mVelocity;

	for (int i = 1; i < nConstraints; i++) {
		C(i, 0) = 0.5 * mParticles[i-1]->mPosition.dot(mParticles[i-1]->mPosition) - mParticles[i-1]->mPosition.dot(mParticles[i]->mPosition) +
			0.5 * mParticles[i]->mPosition.dot(mParticles[i]->mPosition) - 0.5 * distances[i-1] * distances[i-1];
		dC(i, 0) = mParticles[i-1]->mPosition.dot(mParticles[i-1]->mVelocity - mParticles[i]->mVelocity) +
			mParticles[i]->mPosition.dot(mParticles[i]->mVelocity - mParticles[i-1]->mVelocity);

		J.block<1, 3>(i, (i - 1) * 3) = mParticles[i-1]->mPosition - mParticles[i]->mPosition;
		J.block<1, 3>(i, i * 3) = mParticles[i]->mPosition - mParticles[i-1]->mPosition;

		dJ.block<1, 3>(i, (i - 1) * 3) = mParticles[i-1]->mVelocity - mParticles[i]->mVelocity;
		dJ.block<1, 3>(i, i * 3) = mParticles[i]->mVelocity - mParticles[i-1]->mVelocity;
	}

	for (int i = 0; i < mParticles.size(); i++) {
		Q.block<3, 1>(i * 3, 0) = mParticles[i]->mAccumulatedForce;
		q.block<3, 1>(i * 3, 0) = mParticles[i]->mPosition;
		dq.block<3, 1>(i * 3, 0) = mParticles[i]->mVelocity;
	}

	//print_matrices();
}

void MyWorld::calculateConstraints() {
	double ks = 10.0, kd = 1.0;
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