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

		mConstrained = false;
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

	bool mConstrained;			// If this particle is constrained to any other particles ONLY
};

class Constraint {
public:
	typedef enum CONSTRAINT_TYPE {
		CONSTRAINT_CIRCLE,
		CONSTRAINT_DISTANCE,
		CONSTRAINT_VECTOR
	} ConstraintType;

	const Particle * p;
	ConstraintType type;

	Constraint(Particle *p_in) : p(p_in) {}
	Constraint() {}

	virtual double C() const = 0;
	virtual double dC() const = 0;
};

class CircleConstraint : public Constraint {
public:
	double mRadius;
	Eigen::Vector3d mCirclePos;

	CircleConstraint(Particle *p_in, double radius, Eigen::Vector3d position) : Constraint(p_in) {
		this->mRadius = radius;
		this->mCirclePos = position;
	}

	virtual double C() const override;
	virtual double dC() const override;
};

class DistanceConstraint : public Constraint {
public:
	Particle *p_other;
	double distance;

	DistanceConstraint(Particle *p_in, Particle *p_other) : Constraint(p_in) {
		this->p_other = p_other;
		this->distance = sqrt((p_in->mPosition - p_other->mPosition).dot(p_in->mPosition - p_other->mPosition));
	}

	virtual double C() const override;
	virtual double dC() const override;
};

#endif
