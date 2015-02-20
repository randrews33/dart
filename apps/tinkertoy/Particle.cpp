#include "Particle.h"
#include "dart/renderer/RenderInterface.h"
#include <iostream>

using namespace Eigen;

void Particle::draw(dart::renderer::RenderInterface* _ri) {
	if (!_ri)
		return;

	_ri->setPenColor(mColor);

	_ri->pushMatrix();
	_ri->translate(mPosition);
	_ri->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
	_ri->popMatrix();

	iType = IntegratorType::INTEGRATOR_EULER;
}

void Particle::update(float tStep) {
	switch (iType) {
	case INTEGRATOR_EULER:
		Euler(tStep);
		break;
	case INTEGRATOR_RK4:
		RK4(tStep);
		break;
	}
}

void Particle::Euler(float tStep) {
	mVelocity += tStep * (mAccumulatedForce / mMass);
	mPosition += tStep * mVelocity;
}

void Particle::RK4(float tStep) {
	mPosition += tStep * (mVelocity + (mAccumulatedForce / mMass) * tStep / 2);
	mVelocity += tStep * (mAccumulatedForce / mMass);
}

////////////////////////////////////////
// Constraints class
////////////////////////////////////////
double CircleConstraint::C() const {
	return 0.5 * p->mPosition.dot(p->mPosition) - 0.5 * mRadius * mRadius;	// NEEDS TO TAKE INTO ACCOUNT THE POSITION OF THE CIRCLE
}

double CircleConstraint::dC() const {
	return p->mPosition.dot(p->mVelocity);
}

double DistanceConstraint::C() const {
	return 0.5 * p->mPosition.dot(p->mPosition) - p->mPosition.dot(p_other->mPosition) +
		0.5 * p_other->mPosition.dot(p_other->mPosition) - 0.5 * distance * distance;
}

double DistanceConstraint::dC() const {
	return p->mPosition.dot(p->mVelocity - p_other->mVelocity) +
		p_other->mPosition.dot(p_other->mVelocity - p->mVelocity);
}
