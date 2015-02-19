#ifndef _PARTICLE_
#define _PARTICLE_

#include <Eigen/Dense>

namespace dart {
	namespace renderer {
		class RenderInterface;
	}
}

class Particle {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Particle() {
		// Create a default particle
		mMass = 1.0;
		mPosition.setZero();
		mVelocity.setZero();
		mAccumulatedForce.setZero();
		mColor << 0.9, 0.2, 0.2, 1.0;
	}
	virtual ~Particle() {}

	void draw(dart::renderer::RenderInterface* _ri);

	void update(float tStep);
	void Euler(float tStep);
	void RK4(float tStep);

	typedef enum INTEGRATOR_TYPE {
		INTEGRATOR_EULER,
		INTEGRATOR_RK4
	} IntegratorType;

	IntegratorType iType;

	double mMass;
	Eigen::Vector3d mPosition;
	Eigen::Vector3d mVelocity;
	Eigen::Vector3d mAccumulatedForce;

	Eigen::Vector4d mColor;
};

#endif
