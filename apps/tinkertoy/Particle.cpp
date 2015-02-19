#include "Particle.h"
#include "dart/renderer/RenderInterface.h"

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
