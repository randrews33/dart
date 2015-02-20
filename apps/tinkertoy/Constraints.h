#ifndef _CONSTRAINTS_H_
#define _CONSTRAINTS_H_

class Particle;

typedef enum CONSTRANT_TYPE {
	CONSTRIANT_CIRCLE,
	CONSTRAINT_DISTANCE,
	CONSTRAINT_VECTOR
} ConstraintType;

class Constraint {
public:
	const Particle * p;
	ConstraintType type;

	Constraint(Particle *p_in) : p(p_in) {}
	Constraint() {}

	virtual double C() const;
	virtual double dC() const;
};

class CircleConstraint : public Constraint {
public:
	float r;

	CircleConstraint(Particle *p_in, float radius) : Constraint(p_in) {
		this->r = radius;
	}

	double C() const;
	double dC() const;
};

#endif	// _CONSTRAINTS_H_
